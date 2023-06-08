import cv2

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 320x240 @ 30fps


def gstreamer_pipeline(capture_width=320, capture_height=240, framerate=30, pre_proc=True):
    if pre_proc:
        return (
            "v4l2src device=/dev/video0 ! "
            "video/x-raw, "
            "width=(int)%d, height=(int)%d, "
            "framerate=(fraction)%d/1 ! "
            "videoconvert ! "
            "videomedian filtersize=9 ! "
            "videoconvert ! "
            "gaussianblur ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink sync=false drop=true"
            % (
                capture_width,
                capture_height,
                framerate
            )
        )
    else: 
        return (
            "v4l2src device=/dev/video0 ! "
            "video/x-raw, "
            "width=(int)%d, height=(int)%d, "
            "framerate=(fraction)%d/1 ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink sync=false drop=true"
            % (
                capture_width,
                capture_height,
                framerate
            )
        )

        
def start_camera(capture_width=320, capture_height=240, framerate=30, pre_proc=True):
    handle = cv2.VideoCapture(gstreamer_pipeline(capture_width=capture_width, capture_height=capture_height, framerate=framerate, pre_proc=pre_proc), cv2.CAP_GSTREAMER)
    return handle

if __name__ == "__main__":
    pass
