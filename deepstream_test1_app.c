/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <cuda_runtime_api.h>
#include "gstnvdsmeta.h"
#include "gst-nvdssr.h"

#define MAX_DISPLAY_LEN 64
GST_DEBUG_CATEGORY (NVDS_APP);

#define PGIE_CLASS_ID_VEHICLE 0
#define PGIE_CLASS_ID_PERSON 2

/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_OUTPUT_WIDTH 1920
#define MUXER_OUTPUT_HEIGHT 1080

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 40000

#define SR

#ifdef SR
#define SMART_REC_CONTAINER 0
#define VIDEO_CACHE_SIZE 15
#define SMART_REC_DEFAULT_DURATION 10
#define START_TIME 2
#define SMART_REC_DURATION 7
static gboolean bbox_enabled = TRUE;

GOptionEntry entries[] = {
  {"enable", 'e', 0, G_OPTION_ARG_INT, &bbox_enabled,
      "Display Bounding box in recorded files", NULL}
  ,
  {NULL}
  ,
};


NvDsSRContext *ctx = NULL;

static gpointer
smart_record_callback (NvDsSRRecordingInfo * info, gpointer userData)
{
  static GMutex mutex;
  FILE *logfile = NULL;
  g_return_val_if_fail (info, NULL);

  g_mutex_lock (&mutex);
  logfile = fopen ("smart_record.log", "a");
  if (logfile) {
    fprintf (logfile, "%d:%d:%d:%ldms:%s:%s\n",
        info->sessionId, info->width, info->height, info->duration,
        info->dirpath, info->filename);
    fclose (logfile);
  } else {
    g_print ("Error in opeing smart record log file\n");
  }
  g_mutex_unlock (&mutex);

  return NULL;
}

static gboolean
smart_record_event_generator ()
{
  NvDsSRSessionId sessId = 0;
  guint startTime = START_TIME;
  guint duration = SMART_REC_DURATION;

  if (ctx->recordOn) {
    g_print ("Recording done.\n");
    if (NvDsSRStop (ctx, 0) != NVDSSR_STATUS_OK)
      g_printerr ("Unable to stop recording\n");
  } else {
    g_print ("Recording started..\n");
    if (NvDsSRStart (ctx, &sessId, startTime, duration,
            NULL) != NVDSSR_STATUS_OK)
      g_printerr ("Unable to start recording\n");
  }
  return TRUE;
}
#endif


gint frame_number = 0;
gchar pgie_classes_str[4][32] = { "Vehicle", "TwoWheeler", "Person",
  "Roadsign"
};

/* osd_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
 * and update params for drawing rectangle, object information etc. */

static GstPadProbeReturn
osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info,
    gpointer u_data)
{
    GstBuffer *buf = (GstBuffer *) info->data;
    guint num_rects = 0; 
    NvDsObjectMeta *obj_meta = NULL;
    guint vehicle_count = 0;
    guint person_count = 0;
    NvDsMetaList * l_frame = NULL;
    NvDsMetaList * l_obj = NULL;
    NvDsDisplayMeta *display_meta = NULL;
    static bool trig;
    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
      l_frame = l_frame->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);
        int offset = 0;
        for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
                l_obj = l_obj->next) {
            obj_meta = (NvDsObjectMeta *) (l_obj->data);
            if (obj_meta->class_id == PGIE_CLASS_ID_VEHICLE) {
                vehicle_count++;
                num_rects++;
            }
            if (obj_meta->class_id == PGIE_CLASS_ID_PERSON) {
                if(!trig){
                  smart_record_event_generator(); 
                  trig = 1;
                }
                person_count++;
                num_rects++;
            }
        }
        display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
        NvOSD_TextParams *txt_params  = &display_meta->text_params[0];
        display_meta->num_labels = 1;
        txt_params->display_text = g_malloc0 (MAX_DISPLAY_LEN);
        offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Person = %d ", person_count);
        offset = snprintf(txt_params->display_text + offset , MAX_DISPLAY_LEN, "Vehicle = %d ", vehicle_count);

        /* Now set the offsets where the string should appear */
        txt_params->x_offset = 10;
        txt_params->y_offset = 12;

        /* Font , font-color and font-size */
        txt_params->font_params.font_name = "Serif";
        txt_params->font_params.font_size = 10;
        txt_params->font_params.font_color.red = 1.0;
        txt_params->font_params.font_color.green = 1.0;
        txt_params->font_params.font_color.blue = 1.0;
        txt_params->font_params.font_color.alpha = 1.0;

        /* Text background color */
        txt_params->set_bg_clr = 1;
        txt_params->text_bg_clr.red = 0.0;
        txt_params->text_bg_clr.green = 0.0;
        txt_params->text_bg_clr.blue = 0.0;
        txt_params->text_bg_clr.alpha = 1.0;

        nvds_add_display_meta_to_frame(frame_meta, display_meta);
    }

    g_print ("Frame Number = %d Number of objects = %d "
            "Vehicle Count = %d Person Count = %d\n",
            frame_number, num_rects, vehicle_count, person_count);
    frame_number++;
    return GST_PAD_PROBE_OK;
}

static gboolean
bus_call (GstBus * bus, GstMessage * msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;
  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;
    case GST_MESSAGE_ERROR:{
      gchar *debug;
      GError *error;
      gst_message_parse_error (msg, &error, &debug);
      g_printerr ("ERROR from element %s: %s\n",
          GST_OBJECT_NAME (msg->src), error->message);
      if (debug)
        g_printerr ("Error details: %s\n", debug);
      g_free (debug);
      g_error_free (error);
      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

// nvarguscamerasrc -> videoconvert ->nvvideoconver->capsfilter->nvstreammux->nvinfer->nvvideoconvert->nvdsosd->| tee_post_osd->queue_pre_sink->nvegltransform->nveglglessink
//                                                                                                              | tee_post_osd->queue_post_osd->encoder->parser->recordbin
int
main (int argc, char *argv[])
{
  GMainLoop *loop = NULL;
  GstElement *pipeline = NULL, *source = NULL, *h264parser = NULL,
      *decoder = NULL, *streammux = NULL, *sink = NULL, *pgie = NULL, *nvvidconv = NULL,
      *nvosd = NULL;

  GstElement *transform = NULL;
  GstBus *bus = NULL;
  guint bus_watch_id;
  GstPad *osd_sink_pad = NULL;
  GstCaps *caps = NULL, *caps_filter_src = NULL;
  GstElement *vidconvsrc, *nvvidconvsrc, *caps_nvvidconvsrc, *tee_post_osd, *queue_post_osd, *queue_pre_sink;  
  GstElement *encoder_post_osd, *parser_post_osd, *nvvidconv2;

  int current_device = -1;
  cudaGetDevice(&current_device);
  struct cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, current_device);


#ifdef SR
  GOptionContext *gctx = NULL;
  GOptionGroup *group = NULL;
  GError *error = NULL;

  NvDsSRInitParams params = { 0 };

  gctx = g_option_context_new ("Nvidia DeepStream Test-SR app");
  group = g_option_group_new ("SR_test", NULL, NULL, NULL, NULL);
  g_option_group_add_entries (group, entries);

  g_option_context_set_main_group (gctx, group);
  g_option_context_add_group (gctx, gst_init_get_option_group ());

  GST_DEBUG_CATEGORY_INIT (NVDS_APP, "NVDS_APP", 0, NULL);

  if (!g_option_context_parse (gctx, &argc, &argv, &error)) {
    g_printerr ("%s", error->message);
    g_print ("%s", g_option_context_get_help (gctx, TRUE, NULL));
    return -1;
  }
#endif

  /* Standard GStreamer initialization */
  gst_init (&argc, &argv);
  loop = g_main_loop_new (NULL, FALSE);

  /* Create gstreamer elements */
  /* Create Pipeline element that will form a connection of other elements */
  pipeline = gst_pipeline_new ("dstest1-pipeline");

  /* Source element for reading from the file */
  source            = gst_element_factory_make ( "nvarguscamerasrc", "nv-nvarguscamerasrc" );
  vidconvsrc        = gst_element_factory_make ( "videoconvert"    , "convertor_src1"      );
  nvvidconvsrc      = gst_element_factory_make ( "nvvideoconvert"  , "convertor_src2"      );
  caps_nvvidconvsrc = gst_element_factory_make ( "capsfilter"      , "nvmm_caps"           );
  tee_post_osd      = gst_element_factory_make ( "tee"             , "tee-post-osd"        );
  queue_pre_sink    = gst_element_factory_make ( "queue"           , "queue-pre-osd"       );
  nvvidconv2        = gst_element_factory_make ( "nvvideoconvert"  , "nvvideo-converter2"  );

  //TODO: is this right? python does not have this
  caps_filter_src = gst_caps_from_string ("video/x-raw(memory:NVMM), format=NV12, width=1280, height=720, framerate=30/1");
  g_object_set (G_OBJECT (caps_nvvidconvsrc ), "caps", caps_filter_src, NULL);
  gst_caps_unref (caps_filter_src);

  /* Create nvstreammux instance to form batches from one or more sources. */
  streammux = gst_element_factory_make ("nvstreammux", "stream-muxer");

  if (!pipeline || !streammux) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
  }

  /* Use nvinfer to run inferencing on decoder's output,
   * behaviour of inferencing is set through config file */
  pgie = gst_element_factory_make ("nvinfer", "primary-nvinference-engine");

  /* Use convertor to convert from NV12 to RGBA as required by nvosd */
  nvvidconv = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter");

  /* Create OSD to draw on the converted RGBA buffer */
  nvosd = gst_element_factory_make ("nvdsosd", "nv-onscreendisplay");

  /* Finally render the osd output */
  if(prop.integrated) {
    transform = gst_element_factory_make ("nvegltransform", "nvegl-transform");
  }
  sink = gst_element_factory_make ("nveglglessink", "nvvideo-renderer");

  if (!source || !vidconvsrc || !nvvidconvsrc || !caps_nvvidconvsrc || !pgie
      || !nvvidconv || !nvosd || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
  }

  if(!transform && prop.integrated) {
    g_printerr ("One tegra element could not be created. Exiting.\n");
    return -1;
  }

  g_object_set (G_OBJECT (source), "bufapi-version", 1, NULL);
  g_object_set (G_OBJECT (streammux), "width", 1920, NULL);
  g_object_set (G_OBJECT (streammux), "height", 1080, NULL);
  g_object_set (G_OBJECT (streammux), "batch-size", 1, NULL);
  //g_object_set (G_OBJECT (streammux), "batched-push-timeout", 4000000, NULL);

  /* Set all the necessary properties of the nvinfer element,
   * the necessary ones are : */
  g_object_set (G_OBJECT (pgie),
      "config-file-path", "dstest1_pgie_config.txt", NULL);
  g_object_set (G_OBJECT (sink), "sync", 0, NULL);

  /* we add a message handler */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);

  /* Set up the pipeline */
  /* we add all elements into the pipeline */
  if(prop.integrated) {
    gst_bin_add_many (GST_BIN (pipeline),
        source, vidconvsrc, nvvidconvsrc, caps_nvvidconvsrc, streammux, pgie, tee_post_osd, queue_pre_sink,
        nvvidconv, nvosd, nvvidconv2,  transform, sink, NULL);
  }

  GstPad *sinkpad, *srcpad;
  gchar pad_name_sink[16] = "sink_0";
  gchar pad_name_src[16] = "src";

  sinkpad = gst_element_get_request_pad (streammux, pad_name_sink);
  if (!sinkpad) {
    g_printerr ("Streammux request sink pad failed. Exiting.\n");
    return -1;
  }

  srcpad = gst_element_get_static_pad (caps_nvvidconvsrc, pad_name_src);
  if (!srcpad) {
    g_printerr ("Decoder request src pad failed. Exiting.\n");
    return -1;
  }

  if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK) {
      g_printerr ("Failed to link decoder to stream muxer. Exiting.\n");
      return -1;
  }

  gst_object_unref (sinkpad);
  gst_object_unref (srcpad);

  if (!gst_element_link_many (source, vidconvsrc, nvvidconvsrc, caps_nvvidconvsrc, NULL)) {
    g_printerr ("Elements could not be linked: 1. Exiting.\n");
    return -1;
  }

  if (!gst_element_link_many (streammux, pgie,
      nvvidconv, nvosd, nvvidconv2, tee_post_osd, queue_pre_sink, transform, sink, NULL)) {
    g_printerr ("Elements could not be linked: 2. Exiting.\n");
    return -1;
  }
#ifdef SR
  params.containerType = SMART_REC_CONTAINER;
  params.videoCacheSize = VIDEO_CACHE_SIZE;
  params.defaultDuration = SMART_REC_DEFAULT_DURATION;
  params.callback = smart_record_callback;
  params.fileNamePrefix = bbox_enabled ? "With_BBox" : "Without_BBox";

  if (NvDsSRCreate (&ctx, &params) != NVDSSR_STATUS_OK) {
    g_printerr ("Failed to create smart record bin");
    return -1;
  }

  /* Encode the data from tee before recording with bbox */
  encoder_post_osd =
      gst_element_factory_make ("nvv4l2h264enc", "encoder-post-osd");

  /* Parse the encoded data after osd component */
  parser_post_osd = gst_element_factory_make ("h264parse", "parser-post-osd");

  /* Use queue to connect the tee_post_osd to nvencoder */
  queue_post_osd = gst_element_factory_make ("queue", "queue-post-osd");

  gst_bin_add_many (GST_BIN (pipeline), queue_post_osd, encoder_post_osd, 
     parser_post_osd, ctx->recordbin, NULL);

  if (!encoder_post_osd || !parser_post_osd || !queue_post_osd) {
    g_printerr ("One element could not be created. Exiting.\n");
    return -1;
  }

  if (!gst_element_link_many (tee_post_osd, queue_post_osd, encoder_post_osd,
         parser_post_osd, ctx->recordbin, NULL)) {
    g_print ("Elements not linked. Exiting. \n");
    return -1;
  }
  
#endif 

  /* Lets add probe to get informed of the meta data generated, we add probe to
   * the sink pad of the osd element, since by that time, the buffer would have
   * had got all the metadata. */
  osd_sink_pad = gst_element_get_static_pad (nvosd, "sink");
  if (!osd_sink_pad)
    g_print ("Unable to get sink pad\n");
  else
    gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
        osd_sink_pad_buffer_probe, NULL, NULL);
  gst_object_unref (osd_sink_pad);

  /* Set the pipeline to "playing" state */
  g_print ("Now playing: %s\n", argv[1]);
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  /* Wait till pipeline encounters an error or EOS */
  g_print ("Running...\n");
  g_main_loop_run (loop);

  /* Out of the main loop, clean up nicely */
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);
  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));
  g_source_remove (bus_watch_id);
  g_main_loop_unref (loop);
  return 0;
}
