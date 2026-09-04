import os

import cv2
import numpy as np
import pyrealsense2 as rs

from inference_sdk import (
    InferenceHTTPClient,
    InferenceConfiguration
)

WORKSPACE = "eyes-on-inanimate-objects"
WORKFLOW_ID = "xarmvision-vxarmvision-anv01-1-rfdetr-small-t1-logic"

# ROBOFLOW LOCAL SERVER
client = InferenceHTTPClient(
    api_url="http://localhost:9001",
    api_key=os.environ["ROBOFLOW_API_KEY"]
).configure(
    InferenceConfiguration(
        api_key_transport="header"
    )
)

# REALSENSE SETUP
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

print("Starting RealSense pipeline...")

pipeline.start(config)

print("Vision started. Press Ctrl+C or Q to quit.")

align = rs.align(rs.stream.color)


# DEPTH FUNCTION
def get_depth_at_pixel(
    depth_frame,
    x,
    y,
    radius=4
):
    depths = []

    width = depth_frame.get_width()
    height = depth_frame.get_height()

    for dy in range(
        -radius,
        radius + 1
    ):
        for dx in range(
            -radius,
            radius + 1
        ):

            px = x + dx
            py = y + dy

            # Keep pixel inside frame
            if (
                px < 0
                or py < 0
                or px >= width
                or py >= height
            ):
                continue

            distance = depth_frame.get_distance(
                px,
                py
            )

            if distance > 0:
                depths.append(distance)

    if len(depths) == 0:
        return 0.0

    # Median is more robust than mean
    return float(
        np.median(depths)
    )

# MAIN LOOP
try:

    while True:

        # GET REALSENSE FRAMES
        frames = pipeline.wait_for_frames()

        aligned = align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        image = np.asanyarray(
            color_frame.get_data()
        )

        image_height, image_width = (
            image.shape[:2]
        )

        # RUN RF-DETR
        result = client.run_workflow(
            workspace_name=WORKSPACE,
            workflow_id=WORKFLOW_ID,
            images={
                "image": image
            },
            use_cache=True
        )

        output = (
            result[0]
            if isinstance(result, list)
            else result
        )

        prediction_output = output.get(
            "predictions",
            {}
        )

        if isinstance(
            prediction_output,
            dict
        ):
            predictions = (
                prediction_output.get(
                    "predictions",
                    []
                )
            )

        elif isinstance(
            prediction_output,
            list
        ):
            predictions = prediction_output

        else:
            predictions = []

        # CAMERA INTRINSICS
        intrinsics = (
            depth_frame
            .profile
            .as_video_stream_profile()
            .get_intrinsics()
        )

        # PROCESS DETECTIONS
        for pred in predictions:

            name = pred["class"]
            confidence = float(pred["confidence"])

            x = int(round(pred["x"]))
            y = int(round(pred["y"]))
            box_width = int(round(pred["width"]))
            box_height = int(round(pred["height"]))

            # KEEP CENTER INSIDE IMAGE
            x = min(
                max(x, 0),
                image_width - 1
            )

            y = min(
                max(y, 0),
                image_height - 1
            )

            # BOUNDING BOX-
            x1 = max(
                0,
                int(x - box_width / 2)
            )

            y1 = max(
                0,
                int(y - box_height / 2)
            )

            x2 = min(
                image_width - 1,
                int(x + box_width / 2)
            )

            y2 = min(
                image_height - 1,
                int(y + box_height / 2)
            )

            # DEPTH
            depth_m = get_depth_at_pixel(depth_frame, x, y)

            # PIXEL + DEPTH -> CAMERA XYZ
            if depth_m > 0:

                camera_point = (
                    rs.rs2_deproject_pixel_to_point(
                        intrinsics,
                        [x, y],
                        depth_m
                    )
                )

                # meters -> millimeters
                camera_xyz_mm = [
                    camera_point[0] * 1000,
                    camera_point[1] * 1000,
                    camera_point[2] * 1000
                ]

                print(
                    f"class={name} "
                    f"conf={confidence:.3f} "
                    f"depth={depth_m:.3f} m "
                    f"cameraXYZ="
                    f"[{camera_xyz_mm[0]:.1f}, "
                    f"{camera_xyz_mm[1]:.1f}, "
                    f"{camera_xyz_mm[2]:.1f}] mm"
                )

                label = (
                    f"{name} "
                    f"{confidence:.2f} "
                    f"z={depth_m:.3f}m"
                )

            else:

                print(
                    f"class={name} "
                    f"conf={confidence:.3f} "
                    f"depth=NO VALID DEPTH"
                    f"cameraXYZ="
                    f"[{camera_xyz_mm[0]:.1f}, "
                    f"{camera_xyz_mm[1]:.1f}, "
                    f"{camera_xyz_mm[2]:.1f}] mm"
                )

                label = (
                    f"{name} "
                    f"{confidence:.2f} "
                )

            # DRAW BOX
            cv2.rectangle(
                image,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )


            # DRAW DETECTION CENTER
            cv2.circle(
                image,
                (x, y),
                5,
                (0, 0, 255),
                -1
            )

            # DRAW LABEL
            cv2.putText(
                image,
                label,
                (
                    x1,
                    max(20, y1 - 10)
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        cv2.imshow( "D435i + RF-DETR + 3D", image)


        # Q TO EXIT
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), ord("Q")):
            print("Q pressed, quitting...")
            break


except KeyboardInterrupt:
    print("Ctrl+C pressed, quitting...")


finally:

    pipeline.stop()
    cv2.destroyAllWindows()
    print("Vision stopped.")