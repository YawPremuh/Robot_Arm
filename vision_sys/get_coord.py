import cv2
import numpy as np
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

config.enable_stream(
    rs.stream.depth,
    640,
    480,
    rs.format.z16,
    30
)

pipeline.start(config)

align = rs.align(rs.stream.color)

clicked_point = None


def mouse_callback(event, x, y, flags, param):
    global clicked_point

    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)


cv2.namedWindow("Calibration")
cv2.setMouseCallback(
    "Calibration",
    mouse_callback
)

try:
    while True:

        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        image = np.asanyarray(
            color_frame.get_data()
        )

        if clicked_point is not None:

            u, v = clicked_point

            depth_m = depth_frame.get_distance(
                u,
                v
            )

            if depth_m > 0:

                intrinsics = (
                    depth_frame.profile
                    .as_video_stream_profile()
                    .get_intrinsics()
                )

                camera_point = (
                    rs.rs2_deproject_pixel_to_point(
                        intrinsics,
                        [u, v],
                        depth_m
                    )
                )

                camera_xyz_mm = [
                    camera_point[0] * 1000,
                    camera_point[1] * 1000,
                    camera_point[2] * 1000
                ]

                print(
                    f"Pixel: ({u}, {v})"
                )

                print(
                    "Camera XYZ:",
                    [
                        round(camera_xyz_mm[0], 1),
                        round(camera_xyz_mm[1], 1),
                        round(camera_xyz_mm[2], 1)
                    ]
                )

                cv2.circle(
                    image,
                    (u, v),
                    6,
                    (0, 0, 255),
                    -1
                )

            clicked_point = None

        cv2.imshow(
            "Calibration",
            image
        )

        key = cv2.waitKey(1) & 0xFF

        if key in (
            ord("q"),
            ord("Q")
        ):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()