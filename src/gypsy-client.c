/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*- */
/*
 * Gypsy
 *
 * A simple to use and understand GPSD replacement
 * that uses D-Bus, GLib and memory allocations
 *
 * Author: Iain Holmes <iain@gnome.org>
 * Copyright (C) 2007 Iain Holmes
 * Copyright (C) 2007 Openedhand Ltd
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

/*
 * GypsyClient - The GPS connection object that control GPS devices.
 */

#include <config.h>

#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>

#ifdef HAVE_BLUEZ
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>
#endif

#ifdef ENABLE_N810
#include <gpsctrl.h>
#define N810_INTERNAL_GPS_PATH "/dev/pgps"
#endif

#include <glib.h>

#include <dbus/dbus-glib.h>
#include <dbus/dbus-glib-bindings.h>

#include "gypsy-client.h"
#include "gypsy-debug.h"
#include "gypsy-marshal-internal.h"
#include "gypsy-parser.h"
#include "gypsy-garmin-parser.h"
#include "gypsy-nmea-parser.h"

#include "garmin.h"

#define GYPSY_ERROR g_quark_from_static_string ("gypsy-error")

typedef enum {
	GYPSY_DEVICE_TYPE_UNKNOWN = -1,
	GYPSY_DEVICE_TYPE_SERIAL,
	GYPSY_DEVICE_TYPE_GARMIN,
	GYPSY_DEVICE_TYPE_FIFO,
	GYPSY_DEVICE_TYPE_BLUETOOTH
} GypsyDeviceType;

/* Defined in main.c */
extern char* nmea_log;

#define READ_BUFFER_SIZE 1024
#define SPEED_TIMEOUT 1000

typedef struct _GypsyClientPrivate {

	char *device_path; /* Device path of our GPS */
	int fd;	/* File descriptor used to read from the GPS */
	GypsyDeviceType type;

	GIOChannel *channel; /* The channel we talk to the GPS on */
	GIOChannel *debug_log; /* The channel to write the NMEA to,
				  or NULL if debugging is off */

	guint32 error_id, connect_id, input_id;

	GypsyParser *parser;

	/* For serial devices */
	speed_t baudrate;

	/* Fix details */
	int timestamp;
	FixType fix_type;

	/* Position details */
	PositionFields position_fields;
	double latitude;
	double longitude;
	double altitude;

	/* For calculating climb */
	int last_alt_timestamp;

	/* Accuracy details */
	AccuracyFields accuracy_fields;
	double pdop;
	double hdop;
	double vdop;

	/* Course details */
	CourseFields course_fields;
	double speed;
	double direction;
	double climb;

	/* Satellite details */
	int sat_count; /* The known confirmed satellites */
	GypsyClientSatellite satellites[MAX_SAT_SVID];
	int new_sat_count; /* New unconfirmed satellites */
	GypsyClientSatellite new_satellites[MAX_SAT_SVID];
} GypsyClientPrivate;

enum {
	PROP_0,
	PROP_DEVICE,
};

enum {
	ACCURACY_CHANGED,
	POSITION_CHANGED,
	COURSE_CHANGED,
	SATELLITES_CHANGED,
	CONNECTION_CHANGED,
	FIX_STATUS,
	TIME_CHANGED,
	LAST_SIGNAL
};

static guint32 signals[LAST_SIGNAL] = {0, };

G_DEFINE_TYPE (GypsyClient, gypsy_client, G_TYPE_OBJECT);

#define GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GYPSY_TYPE_CLIENT, GypsyClientPrivate))
#define GYPSY_CLIENT_SATELLITES_CHANGED_TYPE (dbus_g_type_get_struct ("GValueArray", G_TYPE_UINT, G_TYPE_BOOLEAN, G_TYPE_UINT, G_TYPE_UINT, G_TYPE_UINT, G_TYPE_INVALID))

static gboolean gypsy_client_set_start_options (GypsyClient *client,
						GHashTable  *options,
						GError     **error);
static gboolean gypsy_client_start (GypsyClient *client,
				    GError     **error);
static gboolean gypsy_client_stop (GypsyClient *client,
				   GError     **error);
static gboolean gypsy_client_get_fix_status (GypsyClient *client,
					     int         *fix_status,
					     GError     **error);
static gboolean gypsy_client_get_connection_status (GypsyClient *client,
						    gboolean    *connected,
						    GError     **error);
static gboolean gypsy_client_get_accuracy (GypsyClient *client,
					   int         *fields_OUT,
					   double      *pdop_OUT,
					   double      *hdop_OUT,
					   double      *vdop_OUT,
					   GError     **error);
static gboolean gypsy_client_get_position (GypsyClient *client,
					   int         *fields_OUT,
					   int         *timestamp_OUT,
					   double      *latitude_OUT,
					   double      *longitude_OUT,
					   double      *altitude_OUT,
					   GError     **error);
static gboolean gypsy_client_get_course (GypsyClient *client,
					 int         *fields_OUT,
					 int         *timestamp_OUT,
					 double      *speed_OUT,
					 double      *direction_OUT,
					 double      *climb_OUT,
					 GError     **error);
static gboolean gypsy_client_get_satellites (GypsyClient *client,
					     GPtrArray  **satellites_OUT,
					     GError     **error);
static gboolean gypsy_client_get_time (GypsyClient *client,
				       int         *timestamp_OUT,
				       GError     **error);

#include "gypsy-client-glue.h"

static void
shutdown_connection (GypsyClient *client)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	if (priv->error_id > 0) {
		g_source_remove (priv->error_id);
		priv->error_id = 0;
	}

	if (priv->connect_id > 0) {
		g_source_remove (priv->connect_id);
		priv->connect_id = 0;
	}

	if (priv->input_id > 0) {
		g_source_remove (priv->input_id);
		priv->input_id = 0;
	}

	if (priv->channel) {
		g_io_channel_shutdown (priv->channel, TRUE, NULL);
		g_io_channel_unref (priv->channel);
		priv->channel = NULL;
	}

	if (priv->debug_log) {
		g_io_channel_shutdown (priv->debug_log, TRUE, NULL);
		g_io_channel_unref (priv->debug_log);
		priv->debug_log = NULL;
	}

	if (priv->fd > 0) {
		close (priv->fd);
		priv->fd = -1;
	}

	if (priv->parser) {
		g_object_unref (priv->parser);
		priv->parser = NULL;
	}

#ifdef ENABLE_N810
	/* Turn off the N810's GPS device */
	if (g_ascii_strcasecmp (priv->device_path, N810_INTERNAL_GPS_PATH) == 0) {
		GYPSY_NOTE (CLIENT, "N810 chip off");
		gpsctrl_set_chip_off ();
	}
#endif /* ENABLE_N810 */
}

static gboolean
gps_channel_error (GIOChannel  *channel,
		   GIOCondition condition,
		   gpointer     userdata)
{
	GypsyClient *client;
	GypsyClientPrivate *priv;

	client = (GypsyClient *) userdata;
	priv = GET_PRIVATE (userdata);

	GYPSY_NOTE (CLIENT, "Error on connection to %s", priv->device_path);
	shutdown_connection ((GypsyClient *) userdata);

	g_signal_emit (G_OBJECT (client), signals[CONNECTION_CHANGED],
		       0, FALSE);

	return FALSE;
}

static gboolean
gps_channel_input (GIOChannel  *channel,
		   GIOCondition condition,
		   gpointer     userdata)
{
	GypsyClientPrivate *priv;
	GIOStatus status;
	char *buf;
	gsize chars_left_in_buffer, chars_read;
	GError *error = NULL;

	priv = GET_PRIVATE (userdata);

	chars_left_in_buffer = gypsy_parser_get_buffer (priv->parser, &buf);
	status = g_io_channel_read_chars (priv->channel,
					  (char *) buf,
					  chars_left_in_buffer,
					  &chars_read,
					  &error);
	if (priv->debug_log) {
		g_io_channel_write_chars (priv->debug_log, buf,
					  chars_read, NULL, NULL);
	}

	if (status == G_IO_STATUS_NORMAL) {
		gypsy_parser_received_data (priv->parser, chars_read, NULL);
	} else {
		GYPSY_NOTE (CLIENT, "Read error on channel %p %d: %s (%s)", priv->channel, status, error->message, g_strerror (errno));
		g_error_free (error);
	}

	return TRUE;
}

/* Returns TRUE/FALSE to indicate success/error */
static gboolean
garmin_usb_device (GIOChannel *channel,
		   char      *devpath,
		   gboolean  *device_is_garmin)
{
	GIOStatus status;
	u_int32_t privcmd[GARMIN_PRIV_PKT_MAX_SIZE];
	gsize chars_written;
	gsize chars_read;

	/*
	 *  Since the Garmin driver sets up the device at /dev/ttyUSB<n> it seems
	 *  to be a pretty good bet that this check will be valid.
	 *
	 *  FIXME: Use gudev to get the VID of the device, and check against Garmin?
	 */

	*device_is_garmin = FALSE;
	if (! strcmp (devpath, "/dev/ttyUSB0")) {

		/* query the device driver to see if it is Garmin */

		privcmd[0] = GARMIN_LAYERID_PRIVATE;		// LayerId
		privcmd[1] = GARMIN_PRIV_PKTID_INFO_REQ;	// PacketId
		privcmd[2] = 0;					// DataLength

		status = g_io_channel_write_chars (channel,
						   (gchar*)privcmd,
						   12,
						   &chars_written,
						   NULL);

		if (status != G_IO_STATUS_NORMAL) {
			g_warning ("GARMIN: Error writing \"Private Info Req\" packet:\n%s",
				   g_strerror (errno));
			return FALSE;
		}

		g_io_channel_flush (channel, NULL);

		/* check the response */

		status = g_io_channel_read_chars (channel,
						  (gchar*)privcmd,
						  GARMIN_PRIV_PKT_INFO_RESP_SIZE,
						  &chars_read,
						  NULL);

		if (status != G_IO_STATUS_NORMAL) {
			g_message ("GARMIN: Error reading \"Private Info Resp\" packet: %s", g_strerror (errno));
			return TRUE;
		}

		if ((privcmd[0] == GARMIN_LAYERID_PRIVATE) &&
		    (privcmd[1] == GARMIN_PRIV_PKTID_INFO_RESP)) {
			/* we're talking to the Garmin driver */
			GYPSY_NOTE (CLIENT, "GARMIN: device type confirmed");
			*device_is_garmin = TRUE;
			return TRUE;
		} else {
			GYPSY_NOTE (CLIENT, "GARMIN: \"Private Info Resp\" packet data not recognized");
			return TRUE;
		}
	}

	return TRUE;
}

static gboolean
garmin_init (GIOChannel *channel)
{
	GIOStatus status;
	u_int32_t privcmd[GARMIN_PRIV_PKT_MAX_SIZE];
	gsize chars_written;
	GError *error = NULL;
	G_Packet_t *pvtpack;

	GYPSY_NOTE (CLIENT, "GARMIN: initialize device");

	/* set the device driver mode to "native" */
	/* (without this, the device will not talk to us) */

	privcmd[0] = GARMIN_LAYERID_PRIVATE;		/* LayerId */
	privcmd[1] = GARMIN_PRIV_PKTID_SET_MODE;	/* PacketId */
	privcmd[2] = 4;					/* DataLength */
	privcmd[3] = GARMIN_MODE_NATIVE;		/* data */

	status = g_io_channel_write_chars (channel,
					   (char*)privcmd,
					   16,
					   &chars_written,
					   NULL);

	if (status != G_IO_STATUS_NORMAL) {
		GYPSY_NOTE (CLIENT, "GARMIN: Error writing \"Private Set Mode\" packet:\n%s", g_strerror (errno));
		g_set_error (&error, GYPSY_ERROR, errno, g_strerror (errno));
		return FALSE;
	}

	g_io_channel_flush (channel, NULL);

	/* start PVT transfers */

	pvtpack = (G_Packet_t*)malloc(sizeof(G_Packet_t)+1);

	pvtpack->mPacketType = LAYERID_APPL;
	pvtpack->mReserved1  = 0;
	pvtpack->mReserved2  = 0;
	pvtpack->mPacketId   = Pid_Command_Data;
	pvtpack->mReserved3  = 0;
	pvtpack->mDataSize   = 2;
	pvtpack->mData[0]    = Cmnd_Start_Pvt_Data;
	pvtpack->mData[1]    = 0;

	status = g_io_channel_write_chars (channel,
					   (gchar*)pvtpack,
					   GARMIN_HEADER_SIZE + pvtpack->mDataSize,
					   &chars_written,
					   NULL);

	if (status != G_IO_STATUS_NORMAL) {
		GYPSY_NOTE (CLIENT, "GARMIN: Error writing \"Start PVT Transfer\" packet:\n%s", g_strerror (errno));
		g_set_error (&error, GYPSY_ERROR, errno, g_strerror (errno));
		return FALSE;
	}

	g_io_channel_flush (channel, NULL);

	free((void*)pvtpack);

	return TRUE;
}

static gboolean
gps_channel_connect (GIOChannel  *channel,
		     GIOCondition condition,
		     gpointer     userdata)
{
	GypsyClientPrivate *priv;
	gboolean ret, device_is_garmin;

	priv = GET_PRIVATE (userdata);

	GYPSY_NOTE (CLIENT, "GPS channel can connect");

	ret = FALSE;
	device_is_garmin = FALSE;
	if (priv->type == GYPSY_DEVICE_TYPE_SERIAL) {
		ret = garmin_usb_device (channel, priv->device_path,
					 &device_is_garmin);
		if (ret == FALSE) {
			/* we got an error trying to figure it out */
			GYPSY_NOTE (CLIENT,
				    "Error determining device type for %s",
				    priv->device_path);
			/* g_set_error () has already been called */
			goto end;
		}
	}

	if (device_is_garmin) {
		priv->type = GYPSY_DEVICE_TYPE_GARMIN;
		priv->parser = gypsy_garmin_parser_new (GYPSY_CLIENT (userdata));
		garmin_init (channel);
	} else {
		priv->parser = gypsy_nmea_parser_new (GYPSY_CLIENT (userdata));
	}

	priv->input_id = g_io_add_watch_full (priv->channel,
					      G_PRIORITY_HIGH_IDLE,
					      G_IO_IN | G_IO_PRI,
					      gps_channel_input,
					      userdata, NULL);

end:
	g_signal_emit (G_OBJECT (userdata), signals[CONNECTION_CHANGED],
		       0, TRUE);

	priv->connect_id = 0;
	return FALSE;
}

static gboolean
gypsy_client_set_start_options (GypsyClient *client,
				GHashTable  *options,
				GError     **error)
{
	GypsyClientPrivate *priv;
	GList *keys, *l;

	priv = GET_PRIVATE (client);

	keys = g_hash_table_get_keys (options);
	for (l = keys; l != NULL; l = l->next) {
		if (g_str_equal (l->data, "BaudRate")) {
			GValue *value = g_hash_table_lookup (options, "BaudRate");
			guint rate;

			if (priv->channel != NULL) {
				g_set_error (error, GYPSY_ERROR, 0, "Device already started");
				g_list_free (keys);
				return FALSE;
			}

			rate = g_value_get_uint (value);
			switch (rate) {
			case 4800:
				priv->baudrate = B4800;
				break;
			case 9600:
				priv->baudrate = B9600;
				break;
			case 19200:
				priv->baudrate = B19200;
				break;
			case 38400:
				priv->baudrate = B38400;
				break;
			case 57600:
				priv->baudrate = B57600;
				break;
			case 115200:
				priv->baudrate = B115200;
				break;
			default:
				GYPSY_NOTE (CLIENT,
					    "Unsupported baud rate '%d'",
					    rate);
				g_set_error (error, GYPSY_ERROR, 0, "Unsupported baud rate '%d'", rate);
				g_list_free (keys);
				return FALSE;
			}
		} else {
			GYPSY_NOTE (CLIENT,
				    "Unsupported option key '%s'", l->data);
		}
	}
	g_list_free (keys);

	return TRUE;
}

static gboolean
gypsy_client_start (GypsyClient *client,
		    GError     **error)
{
	GypsyClientPrivate *priv;
	GIOStatus status;

	priv = GET_PRIVATE (client);

	if (priv->fd != -1) {
		GYPSY_NOTE (CLIENT, "Connection to %s already started",
			       priv->device_path);
		return TRUE;
	}
	else {
		GYPSY_NOTE (CLIENT, "Starting connection to %s",
			       priv->device_path);
	}

	/* Enable the N810's internal GPS */
#ifdef ENABLE_N810
		if (g_ascii_strcasecmp (priv->device_path, N810_INTERNAL_GPS_PATH) == 0) {
			GYPSY_NOTE (CLIENT, "N810 chip on");
			gpsctrl_set_chip_on ();
		}
#endif /* ENABLE_N810 */

	/* Open a connection to our device */

	/* we assume that a device path starting with slash is a tty device or
	 * a FIFO */
	if (priv->device_path[0] == '/') {
		priv->fd = open (priv->device_path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
		if (priv->fd != -1 && isatty (priv->fd)) {
			/* Reopen read-write for TTY devices,
			 * we'll be detecting whether it's garmin later */
			priv->type = GYPSY_DEVICE_TYPE_SERIAL;
			close (priv->fd);
			priv->fd = open (priv->device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
		} else {
			priv->type = GYPSY_DEVICE_TYPE_FIFO;
		}

		if (priv->fd == -1) {
			g_warning ("Error opening device %s: %s", priv->device_path, g_strerror (errno));
			g_set_error (error, GYPSY_ERROR, errno, g_strerror (errno));
			return FALSE;
		}

		if (priv->type == GYPSY_DEVICE_TYPE_SERIAL) {
			struct termios term;

			if (tcgetattr (priv->fd, &term) < 0) {
				g_warning ("Error getting term: %s", g_strerror (errno));
				g_set_error (error, GYPSY_ERROR, errno, g_strerror (errno));
				return FALSE;
			}

			cfmakeraw(&term);
			if (priv->baudrate != B0)
				cfsetispeed(&term, priv->baudrate);

			if (tcsetattr (priv->fd, TCSAFLUSH, &term) < 0) {
				g_warning ("Error setting term: %s", g_strerror (errno));
				g_set_error (error, GYPSY_ERROR, errno, g_strerror (errno));
				return FALSE;
			}
		}
	} else {
		priv->type = GYPSY_DEVICE_TYPE_BLUETOOTH;
#ifdef HAVE_BLUEZ
		priv->fd = socket (AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
#else
		g_warning ("Trying to connect to a Bluetooth GPS but Gypsy does not have Bluetooth support");
		return FALSE;
#endif
	}

	if (nmea_log) {
		if (g_str_equal (nmea_log, "stdout") ||
		    g_str_equal (nmea_log, "-")) {
			priv->debug_log = g_io_channel_unix_new (STDOUT_FILENO);
		} else {
			char *device, *filename;
			device = g_path_get_basename (priv->device_path);
			filename = g_strconcat (nmea_log, ".", device, NULL);
			priv->debug_log = g_io_channel_new_file (filename, "w", NULL);
			g_free (device);
			g_free (filename);
		}
		g_io_channel_set_encoding (priv->debug_log, NULL, NULL);
		g_io_channel_set_buffered (priv->debug_log, FALSE);
	}

	/* Set up the IO Channel */

	priv->channel = g_io_channel_unix_new (priv->fd);
	GYPSY_NOTE (CLIENT, "Created new channel: %p", priv->channel);

	status = g_io_channel_set_flags (priv->channel,
					 G_IO_FLAG_NONBLOCK | G_IO_FLAG_IS_READABLE | G_IO_FLAG_IS_WRITEABLE,
					 NULL);

	if (status != G_IO_STATUS_NORMAL) {
		g_warning ("Error setting flags: %s", g_strerror (errno));
		g_set_error (error, GYPSY_ERROR, errno, g_strerror (errno));
		return FALSE;
	}

	/* set encoding to NULL so we can do reads and writes */
	status = g_io_channel_set_encoding (priv->channel, NULL, NULL);

	if (status != G_IO_STATUS_NORMAL) {
		g_warning ("Error setting encoding: %s", g_strerror (errno));
		g_set_error (error, GYPSY_ERROR, errno, g_strerror (errno));
		return FALSE;
	}

	priv->error_id = g_io_add_watch_full (priv->channel,
					      G_PRIORITY_HIGH_IDLE,
					      G_IO_ERR | G_IO_HUP,
					      gps_channel_error, client, NULL);

	priv->connect_id = g_io_add_watch_full (priv->channel,
						G_PRIORITY_HIGH_IDLE,
						G_IO_OUT,
						gps_channel_connect, client, NULL);

#ifdef HAVE_BLUEZ
	/* Now connect to the bluetooth socket */
	if (priv->device_path[0] != '/') {
		struct sockaddr_rc addr = { 0 };

		addr.rc_family = AF_BLUETOOTH;
		addr.rc_channel = (uint8_t) 1;
		str2ba(priv->device_path, &addr.rc_bdaddr);

		if (connect (priv->fd, (struct sockaddr *) &addr, sizeof (addr)) == -1) {
			/* Error */
			if (errno == EINPROGRESS || errno == EAGAIN) {
				return TRUE;
			}

			g_warning ("Error connecting: %s", g_strerror (errno));
			g_set_error (error, GYPSY_ERROR, errno, g_strerror (errno));

			g_source_remove (priv->error_id);
			priv->error_id = 0;
			g_source_remove (priv->connect_id);
			priv->connect_id = 0;

			g_io_channel_unref (priv->channel);
			priv->channel = NULL;

			close (priv->fd);
			priv->fd = -1;

			return FALSE;
		}
	}
#endif
	return TRUE;
}

static gboolean
gypsy_client_stop (GypsyClient *client,
		   GError     **error)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	GYPSY_NOTE (CLIENT, "Stopping connection to %s", priv->device_path);
	shutdown_connection (client);

	g_signal_emit (G_OBJECT (client), signals[CONNECTION_CHANGED],
		       0, FALSE);
	return TRUE;
}

static gboolean
gypsy_client_get_connection_status (GypsyClient *client,
				    gboolean    *connected,
				    GError     **error)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	*connected = (priv->fd > 0);

	return TRUE;
}

static gboolean
gypsy_client_get_fix_status (GypsyClient *client,
			     int         *fix_status,
			     GError     **error)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	*fix_status = priv->fix_type;

	return TRUE;
}

static gboolean 
gypsy_client_get_accuracy (GypsyClient *client,
			   int         *fields_OUT,
			   double      *pdop_OUT,
			   double      *hdop_OUT,
			   double      *vdop_OUT,
			   GError     **error)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	*fields_OUT = priv->accuracy_fields;
	*pdop_OUT = priv->pdop;
	*hdop_OUT = priv->hdop;
	*vdop_OUT = priv->vdop;

	return TRUE;
}

static gboolean 
gypsy_client_get_position (GypsyClient *client,
			   int         *fields_OUT,
			   int         *timestamp_OUT,
			   double      *latitude_OUT,
			   double      *longitude_OUT,
			   double      *altitude_OUT,
			   GError     **error)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	*fields_OUT = priv->position_fields;
	*timestamp_OUT = priv->timestamp;
	*latitude_OUT = priv->latitude;
	*longitude_OUT = priv->longitude;
	*altitude_OUT = priv->altitude;

	return TRUE;
}

static gboolean 
gypsy_client_get_course (GypsyClient *client,
			 int         *fields_OUT,
			 int         *timestamp_OUT,
			 double      *speed_OUT,
			 double      *direction_OUT,
			 double      *climb_OUT,
			 GError     **error)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	*fields_OUT = priv->position_fields;
	*timestamp_OUT = priv->timestamp;
	*speed_OUT = priv->speed;
	*direction_OUT = priv->direction;
	*climb_OUT = priv->climb;

	return TRUE;
}

static gboolean
gypsy_client_get_satellites (GypsyClient *client,
			     GPtrArray  **satellites_OUT,
			     GError     **error)
{
	GypsyClientPrivate *priv;
	GPtrArray *sat_array;
	int i;

	priv = GET_PRIVATE (client);

	sat_array = g_ptr_array_new ();

	for (i = 0; i < priv->sat_count; i++) {
		GValue sat_struct = {0, };
		GypsyClientSatellite *sat;

		sat = &priv->satellites[i];

		g_value_init (&sat_struct,
			      GYPSY_CLIENT_SATELLITES_CHANGED_TYPE);
		g_value_take_boxed (&sat_struct,
				    dbus_g_type_specialized_construct
				    (GYPSY_CLIENT_SATELLITES_CHANGED_TYPE));

		dbus_g_type_struct_set (&sat_struct,
					0, sat->satellite_id,
					1, sat->in_use,
					2, sat->elevation,
					3, sat->azimuth,
					4, sat->snr,
					G_MAXUINT);
		g_ptr_array_add (sat_array, 
				 g_value_get_boxed (&sat_struct));
	}

	*satellites_OUT = sat_array;

	return TRUE;
}

static gboolean
gypsy_client_get_time (GypsyClient *client,
		       int         *timestamp_OUT,
		       GError     **error)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	*timestamp_OUT = priv->timestamp;

	return TRUE;
}

static void
finalize (GObject *object) 
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (object);

	shutdown_connection ((GypsyClient *) object);

	g_free (priv->device_path);

	((GObjectClass *) gypsy_client_parent_class)->finalize (object);
}

static void
dispose (GObject *object)
{
	((GObjectClass *) gypsy_client_parent_class)->dispose (object);
}

static void
set_property (GObject      *object,
	      guint         prop_id,
	      const GValue *value,
	      GParamSpec   *pspec)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (object);

	switch (prop_id) {
	case PROP_DEVICE:
		priv->device_path = g_value_dup_string (value);

		break;

	default:
		break;
	}
}

static void
get_property (GObject    *object,
	      guint       prop_id,
	      GValue     *value,
	      GParamSpec *pspec)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (object);

	switch (prop_id) {
	case PROP_DEVICE:
		g_value_set_string (value, priv->device_path);
		break;

	default:
		break;
	}
}

static void
gypsy_client_class_init (GypsyClientClass *klass)
{
	GObjectClass *o_class;

	o_class = (GObjectClass *) klass;

	o_class->finalize = finalize;
	o_class->dispose = dispose;
	o_class->set_property = set_property;
	o_class->get_property = get_property;

	g_type_class_add_private (klass, sizeof (GypsyClientPrivate));

	g_object_class_install_property (o_class,
					 PROP_DEVICE,
					 g_param_spec_string ("device_path",
							      "Device path", 
							      "The path of the GPS device",
							      "", 
							      G_PARAM_WRITABLE |
							      G_PARAM_CONSTRUCT_ONLY));

	signals[ACCURACY_CHANGED] = g_signal_new ("accuracy-changed",
						  G_TYPE_FROM_CLASS (klass),
						  G_SIGNAL_RUN_FIRST |
						  G_SIGNAL_NO_RECURSE,
						  G_STRUCT_OFFSET (GypsyClientClass, 
								   accuracy_changed),
						  NULL, NULL,
						  gypsy_marshal_VOID__INT_DOUBLE_DOUBLE_DOUBLE,
						  G_TYPE_NONE,
						  4, G_TYPE_INT,
						  G_TYPE_DOUBLE, G_TYPE_DOUBLE, G_TYPE_DOUBLE);
	signals[POSITION_CHANGED] = g_signal_new ("position-changed",
						  G_TYPE_FROM_CLASS (klass),
						  G_SIGNAL_RUN_FIRST |
						  G_SIGNAL_NO_RECURSE,
						  G_STRUCT_OFFSET (GypsyClientClass, 
								   position_changed),
						  NULL, NULL,
						  gypsy_marshal_VOID__INT_INT_DOUBLE_DOUBLE_DOUBLE,
						  G_TYPE_NONE,
						  5, G_TYPE_INT,
						  G_TYPE_INT, G_TYPE_DOUBLE,
						  G_TYPE_DOUBLE, G_TYPE_DOUBLE);
	signals[COURSE_CHANGED] = g_signal_new ("course-changed",
						G_TYPE_FROM_CLASS (klass),
						G_SIGNAL_RUN_FIRST |
						G_SIGNAL_NO_RECURSE,
						G_STRUCT_OFFSET (GypsyClientClass, 
								 course_changed),
						NULL, NULL,
						gypsy_marshal_VOID__INT_INT_DOUBLE_DOUBLE_DOUBLE,
						G_TYPE_NONE,
						5, G_TYPE_INT,
						G_TYPE_INT, G_TYPE_DOUBLE,
						G_TYPE_DOUBLE, G_TYPE_DOUBLE);
	signals[SATELLITES_CHANGED] = g_signal_new ("satellites-changed",
						    G_TYPE_FROM_CLASS (klass),
						    G_SIGNAL_RUN_LAST, 0,
						    NULL, NULL,
						    g_cclosure_marshal_VOID__BOXED,
						    G_TYPE_NONE, 1, 
						    (dbus_g_type_get_collection
						     ("GPtrArray", 
						      (dbus_g_type_get_struct
						       ("GValueArray", 
							G_TYPE_UINT, 
							G_TYPE_BOOLEAN,
							G_TYPE_UINT,
							G_TYPE_UINT,
							G_TYPE_UINT,
							G_TYPE_INVALID)))));
						    
	signals[CONNECTION_CHANGED] = g_signal_new ("connection-status-changed",
						    G_TYPE_FROM_CLASS (klass),
						    G_SIGNAL_RUN_FIRST |
						    G_SIGNAL_NO_RECURSE,
						    G_STRUCT_OFFSET (GypsyClientClass, connection_changed),
						    NULL, NULL,
						    g_cclosure_marshal_VOID__BOOLEAN,
						    G_TYPE_NONE,
						    1, G_TYPE_BOOLEAN);
	signals[FIX_STATUS] = g_signal_new ("fix-status-changed",
					    G_TYPE_FROM_CLASS (klass),
					    G_SIGNAL_RUN_FIRST |
					    G_SIGNAL_NO_RECURSE,
					    G_STRUCT_OFFSET (GypsyClientClass,
							     fix_status_changed),
					    NULL, NULL,
					    g_cclosure_marshal_VOID__INT,
					    G_TYPE_NONE, 
					    1, G_TYPE_INT);
	signals[TIME_CHANGED] = g_signal_new ("time-changed",
					      G_TYPE_FROM_CLASS (klass),
					      G_SIGNAL_RUN_FIRST |
					      G_SIGNAL_NO_RECURSE,
					      G_STRUCT_OFFSET (GypsyClientClass,
							       time_changed),
					      NULL, NULL,
					      g_cclosure_marshal_VOID__INT,
					      G_TYPE_NONE,
					      1, G_TYPE_INT);

	dbus_g_object_type_install_info (G_TYPE_FROM_CLASS (klass),
					 &dbus_glib_gypsy_client_object_info);
}

static void
gypsy_client_init (GypsyClient *client)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);
	priv->fd = -1;
	priv->type = GYPSY_DEVICE_TYPE_UNKNOWN;
	priv->baudrate = B0;
	priv->timestamp = 0;
	priv->last_alt_timestamp = 0;
	priv->parser = NULL;
}

void
gypsy_client_set_position (GypsyClient   *client,
			   PositionFields fields_set,
			   float          latitude,
			   float          longitude,
			   float          altitude)
{
	GypsyClientPrivate *priv;
	gboolean changed = FALSE;

	priv = GET_PRIVATE (client);

	if (fields_set & POSITION_LATITUDE) {
		if (priv->position_fields & POSITION_LATITUDE) {
			if (priv->latitude != latitude) {
				priv->latitude = latitude;
				changed = TRUE;
			}
		} else {
			priv->latitude = latitude;
			priv->position_fields |= POSITION_LATITUDE;
			changed = TRUE;
		}
	}

	if (fields_set & POSITION_LONGITUDE) {
		if (priv->position_fields & POSITION_LONGITUDE) {
			if (priv->longitude != longitude) {
				priv->longitude = longitude;
				changed = TRUE;
			}
		} else {
				priv->longitude = longitude;
				priv->position_fields |= POSITION_LONGITUDE;
				changed = TRUE;
		}
	}

	if (fields_set & POSITION_ALTITUDE) {
		if (priv->position_fields & POSITION_ALTITUDE) {
			if (priv->altitude != altitude) {
				/* If we've got a timestamp for the last alt
				   then we are able to calculate the climb. */
				if (priv->last_alt_timestamp > 0) {
					int dt;
					double da, climb;

					dt = priv->timestamp - priv->last_alt_timestamp;
					da = altitude - priv->altitude;
					climb = da / (double) dt;
					gypsy_client_set_course (client,
								 COURSE_CLIMB,
								 0.0, 0.0, 
								 climb);
				}

				priv->altitude = altitude;
				priv->last_alt_timestamp = priv->timestamp;
				changed = TRUE;
			} 
		} else {
			priv->altitude = altitude;
			priv->last_alt_timestamp = priv->timestamp;
			priv->position_fields |= POSITION_ALTITUDE;
			changed = TRUE;
		}
	}

	if (changed) {
		g_signal_emit (client, signals[POSITION_CHANGED], 0,
			       priv->position_fields, priv->timestamp, 
			       priv->latitude, priv->longitude, priv->altitude);
	}
}

void
gypsy_client_set_course (GypsyClient *client,
			 CourseFields fields_set,
			 float        speed,
			 float        direction,
			 float        climb)
{
	GypsyClientPrivate *priv;
	gboolean changed = FALSE;

	priv = GET_PRIVATE (client);

	if (fields_set & COURSE_SPEED) {
		if (priv->course_fields & COURSE_SPEED) {
			if (priv->speed != speed) {
				priv->speed = speed;
				changed = TRUE;
			}
		} else {
			priv->speed = speed;
			priv->course_fields |= COURSE_SPEED;
			changed = TRUE;
		}
	}

	if (fields_set & COURSE_DIRECTION) {
		if (priv->course_fields & COURSE_DIRECTION) {
			if (priv->direction != direction) {
				priv->direction = direction;
				changed = TRUE;
			}
		} else {
			priv->direction = direction;
			priv->course_fields |= COURSE_DIRECTION;
			changed = TRUE;
		}
	}

	if (fields_set & COURSE_CLIMB) {
		if (priv->course_fields & COURSE_CLIMB) {
			if (priv->climb != climb) {
				priv->climb = climb;
				changed = TRUE;
			}
		} else {
			priv->climb = climb;
			priv->course_fields |= COURSE_CLIMB;
			changed = TRUE;
		}
	}

	if (changed) {
		/* Emit course changed, we don't deal with climb yet */
		g_signal_emit (client, signals[COURSE_CHANGED], 0,
			       priv->course_fields, priv->timestamp, 
			       priv->speed, priv->direction, priv->climb);
	}
}

void
gypsy_client_set_timestamp (GypsyClient *client,
			    int          utc_time)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	if (priv->timestamp != utc_time) {
		priv->timestamp = utc_time;
		g_signal_emit (client, signals[TIME_CHANGED], 0, utc_time);
	}
}

void
gypsy_client_set_fix_type (GypsyClient *client,
			   FixType      type,
			   gboolean     weak)
{
	GypsyClientPrivate *priv;
	FixType weak_type;

	priv = GET_PRIVATE (client);

	/* If the passed in type is "weak" (in an RMC sentence we only 
	   know if we have a fix or not) then we don't want to demote a 3D 
	   fix down to a 2D fix only to have it promoted 2 sentences later.
	   So we convert a 3D fix to a 2D one before checking */
	if (weak && priv->fix_type == FIX_3D) {
		weak_type = FIX_2D;
	} else {
		weak_type = priv->fix_type;
	}

	if (weak_type != type) {
		priv->fix_type = type;
		g_signal_emit (G_OBJECT (client), signals[FIX_STATUS], 0, type);
	}
}

void
gypsy_client_set_accuracy (GypsyClient *client,
			   AccuracyFields fields_set,
			   double pdop,
			   double hdop,
			   double vdop)
{
	GypsyClientPrivate *priv;
	gboolean changed = FALSE;

	priv = GET_PRIVATE (client);

	if (fields_set & ACCURACY_POSITION) {
		if (priv->accuracy_fields & ACCURACY_POSITION) {
			if (priv->pdop != pdop) {
				priv->pdop = pdop;
				changed = TRUE;
			}
		} else {
			priv->pdop = pdop;
			priv->accuracy_fields |= ACCURACY_POSITION;
			changed = TRUE;
		}
	}

	if (fields_set & ACCURACY_HORIZONTAL) {
		if (priv->accuracy_fields & ACCURACY_HORIZONTAL) {
			if (priv->hdop != hdop) {
				priv->hdop = hdop;
				changed = TRUE;
			}
		} else {
			priv->hdop = hdop;
			priv->accuracy_fields |= ACCURACY_HORIZONTAL;
			changed = TRUE;
		}
	}

	if (fields_set & ACCURACY_VERTICAL) {
		if (priv->accuracy_fields & ACCURACY_VERTICAL) {
			if (priv->vdop != vdop) {
				priv->vdop = vdop;
				changed = TRUE;
			}
		} else {
			priv->vdop = vdop;
			priv->accuracy_fields |= ACCURACY_VERTICAL;
			changed = TRUE;
		}
	}

	if (changed) {
		g_signal_emit (client, signals[ACCURACY_CHANGED], 0,
			       priv->accuracy_fields,
			       priv->pdop, priv->hdop, priv->vdop);
	}
}

/* This adds a satellite to the new set of satellites.
   Once all the satellites are set, call gypsy_client_set_satellites
   to commit them */
void 
gypsy_client_add_satellite (GypsyClient *client,
			    int          satellite_id,
			    gboolean     in_use,
			    int          elevation,
			    int          azimuth,
			    int          snr)
{
	GypsyClientPrivate *priv;
	GypsyClientSatellite *satellite;

	priv = GET_PRIVATE (client);

	satellite = &(priv->new_satellites[priv->new_sat_count]);
	satellite->satellite_id = satellite_id;
	satellite->in_use = in_use;
	satellite->elevation = elevation;
	satellite->azimuth = azimuth;
	satellite->snr = snr;

	priv->new_sat_count++;
}

/* This is called if there was an error in the satellites
   like a missing message. If this happens then we just clear the ones
   we have and continue */
void 
gypsy_client_clear_satellites (GypsyClient *client)
{
	GypsyClientPrivate *priv;

	priv = GET_PRIVATE (client);

	priv->new_sat_count = 0;
}

/* Checks if the satellite details have changed, and if so copies the new
   set over the old and emits a signal */
void
gypsy_client_set_satellites (GypsyClient *client) 
{
	GypsyClientPrivate *priv;
	gboolean changed = FALSE;
	int i;

	priv = GET_PRIVATE (client);

	for (i = 0; i < priv->new_sat_count; i++) {
		GypsyClientSatellite *n, *o;

		n = &priv->new_satellites[i];
		o = &priv->satellites[i];

		if (n->satellite_id != o->satellite_id ||
		    n->in_use != o->in_use ||
		    n->elevation != o->elevation ||
		    n->azimuth != o->azimuth ||
		    n->snr != o->snr) {
			changed = TRUE;
			o->satellite_id = n->satellite_id;
			o->in_use = n->in_use;
			o->elevation = n->elevation;
			o->azimuth = n->azimuth;
			o->snr = n->snr;
		}
	}

	if (priv->new_sat_count != priv->sat_count) {
		changed = TRUE;
		priv->sat_count = priv->new_sat_count;
	}

	if (changed) {
		GPtrArray *sat_array;
		int i;

		sat_array = g_ptr_array_new ();
		for (i = 0; i < priv->sat_count; i++) {
			GValue sat_struct = {0, };
			GypsyClientSatellite *sat;

			sat = &priv->satellites[i];

			g_value_init (&sat_struct,
				      GYPSY_CLIENT_SATELLITES_CHANGED_TYPE);
			g_value_take_boxed (&sat_struct,
					    dbus_g_type_specialized_construct
					    (GYPSY_CLIENT_SATELLITES_CHANGED_TYPE));

			dbus_g_type_struct_set (&sat_struct,
						0, sat->satellite_id,
						1, sat->in_use,
						2, sat->elevation,
						3, sat->azimuth,
						4, sat->snr,
						G_MAXUINT);
			g_ptr_array_add (sat_array, 
					 g_value_get_boxed (&sat_struct));
		}

		g_signal_emit (client, signals[SATELLITES_CHANGED], 0, 
			       sat_array);

		for (i = 0; i < sat_array->len; i++) {
			g_boxed_free (GYPSY_CLIENT_SATELLITES_CHANGED_TYPE,
				      g_ptr_array_index (sat_array, i));
		}
		g_ptr_array_free (sat_array, TRUE);
	}

	priv->new_sat_count = 0;
}

