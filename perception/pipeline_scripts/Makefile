BIN_CLOUDS = ~/b/perception/clouds
BIN_PERCEPTION = ~/b/perception
IMG_PROC = ~/python/image_proc

ONCE = $(DATA_ROOT)/once
TABLE_CORNERS = $(ONCE)/table_corners.txt
ROI_MASK = $(ONCE)/roi_mask.png
FIRST_PCD = $(DATA_ROOT)/kinect/data000000000000.pcd
FIRST_LABEL = $(DATA_ROOT)/labels/data000000000000.png
FIRST_IMAGE = $(DATA_ROOT)/images/data000000000000.jpg
INIT_ROPE = $(ONCE)/init_rope.txt

KINECT = $(DATA_ROOT)/kinect
IMAGES = $(DATA_ROOT)/images
LABELS = $(DATA_ROOT)/labels
ROPE_PTS = $(DATA_ROOT)/rope_pts
ROPE_ENDS = $(DATA_ROOT)/rope_ends
TOWEL_PTS = $(DATA_ROOT)/towel_pts

setup : $(TABLE_CORNERS) $(ROI_MASK)
before_train : $(ROI_MASK) $(IMAGES)
playback : PLAYBACK
robot_knot : ROBOT_KNOT
human_knot : HUMAN_KNOT
robot_towel : ROBOT_TOWEL
human_towel : HUMAN_TOWEL

$(TABLE_CORNERS) : $(FIRST_PCD)
	$(BIN_CLOUDS)/comm_get_table -i $(FIRST_PCD)
$(ROI_MASK) : $(FIRST_PCD)
	$(IMG_PROC)/make_roi.py -p $(FIRST_PCD) -o $(ONCE)/roi_mask.png
$(FIRST_LABEL) : $(LABELS)
$(FIRST_IMAGE) : $(IMAGES)
$(FIRST_PCD) : $(KINECT)
$(KINECT) : 
	$(BIN_CLOUDS)/comm_write_pcds -d 4
$(IMAGES) : $(KINECT)
	$(IMG_PROC)/comm_pcd2jpg.py kinect images
$(LABELS) : $(IMAGES) $(CLASSIFIER) $(ROI_MASK)
	$(IMG_PROC)/comm_apply_classifier.py images labels $(CLASSIFIER) --mask=$(ROI_MASK)
$(ROPE_PTS) :
	$(BIN_CLOUDS)/comm_downsample_clouds -c kinect -l labels -o rope_pts -n 1 -v .01
$(TOWEL_PTS) :
	$(BIN_CLOUDS)/comm_downsample_clouds -c kinect -l labels -o towel_pts -n 1 -v .02
$(SPONGE_PTS) : 
	$(BIN_CLOUDS)/comm_downsample_clouds -c kinect -l labels -o towel_pts -n 1 -v .01

$(ROPE_ENDS) : 
	$(IMG_PROC)/comm_get_region_centers.py labels kinect rope_ends 2 --min_pix=25 --max_blobs=2
PLAYBACK :
	$(BIN_PERCEPTION)/test_pr2_playback
ROBOT_KNOT : $(LABELS) $(ROPE_PTS) $(ROPE_ENDS) $(TABLE_CORNERS) $(INIT_ROPE)
	$(BIN_PERCEPTION)/test_pr2_knot
HUMAN_KNOT : $(LABELS) $(ROPE_PTS) $(ROPE_ENDS) $(TABLE_CORNERS) $(INIT_ROPE)
	$(BIN_PERCEPTION)/test_kinect_rope
ROBOT_TOWEL : $(LABELS) $(TOWEL_PTS) $(TABLE_CORNERS)

HUMAN_TOWEL : $(LABELS) $(TOWEL_PTS) $(TABLE_CORNERS)
