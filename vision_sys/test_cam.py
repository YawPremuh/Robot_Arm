import cv2
import numpy as np
import pyrealsense2 as rs


# ============================================================
# REALSENSE SETUP
# ============================================================

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

print("Starting RealSense...")

profile = pipeline.start(config)

print("RealSense started.")


# ============================================================
# ALIGN DEPTH TO COLOR
# ============================================================

align = rs.align(rs.stream.color)


# ============================================================
# DEPTH FILTERS
# ============================================================

spatial_filter = rs.spatial_filter()
temporal_filter = rs.temporal_filter()
hole_filling_filter = rs.hole_filling_filter()


# ============================================================
# CLICK HANDLING
# ============================================================

clicked_point = None


def mouse_callback(event, x, y, flags, param):
    global clicked_point

    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)


cv2.namedWindow("Depth Calibration Test")

cv2.setMouseCallback(
    "Depth Calibration Test",
    mouse_callback
)


# ============================================================
# GET DEPTH STATISTICS AROUND CLICKED PIXEL
# ============================================================

def get_depth_patch(
    depth_frame,
    x,
    y,
    radius=4
):
    values = []

    width = depth_frame.get_width()
    height = depth_frame.get_height()

    for dy in range(-radius, radius + 1):

        for dx in range(-radius, radius + 1):

            px = x + dx
            py = y + dy

            # Keep coordinates inside image
            if (
                px < 0
                or py < 0
                or px >= width
                or py >= height
            ):
                continue

            depth_m = depth_frame.get_distance(
                px,
                py
            )

            if depth_m > 0:
                values.append(depth_m)

    if not values:
        return None

    values = np.array(
        values,
        dtype=float
    )

    return {
        "median": float(np.median(values)),
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "count": len(values)
    }


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        # ----------------------------------------------------
        # GET FRAMES
        # ----------------------------------------------------

        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(
            frames
        )

        color_frame = (
            aligned_frames
            .get_color_frame()
        )

        depth_frame = (
            aligned_frames
            .get_depth_frame()
        )

        if not color_frame or not depth_frame:
            continue


        # ----------------------------------------------------
        # APPLY DEPTH FILTERS
        # ----------------------------------------------------

        filtered_depth = spatial_filter.process(
            depth_frame
        )

        filtered_depth = temporal_filter.process(
            filtered_depth
        )

        filtered_depth = hole_filling_filter.process(
            filtered_depth
        )

        # IMPORTANT:
        # Convert generic frame back into a depth_frame
        depth_frame = filtered_depth.as_depth_frame()

        if not depth_frame:
            continue


        # ----------------------------------------------------
        # COLOR IMAGE
        # ----------------------------------------------------

        image = np.asanyarray(
            color_frame.get_data()
        )


        # ----------------------------------------------------
        # PROCESS CLICK
        # ----------------------------------------------------

        if clicked_point is not None:

            x, y = clicked_point

            stats = get_depth_patch(
                depth_frame,
                x,
                y,
                radius=4
            )

            if stats is None:

                print(
                    "\nNo valid depth values "
                    "at clicked point."
                )

            else:

                # Use median because it is more robust
                depth_m = stats["median"]

                # --------------------------------------------
                # GET CAMERA INTRINSICS
                # --------------------------------------------

                intrinsics = (
                    depth_frame
                    .profile
                    .as_video_stream_profile()
                    .get_intrinsics()
                )


                # --------------------------------------------
                # PIXEL + DEPTH -> CAMERA XYZ
                # --------------------------------------------

                camera_point = (
                    rs.rs2_deproject_pixel_to_point(
                        intrinsics,
                        [x, y],
                        depth_m
                    )
                )


                # RealSense gives meters
                # Convert XYZ to millimeters
                camera_xyz_mm = [
                    camera_point[0] * 1000,
                    camera_point[1] * 1000,
                    camera_point[2] * 1000
                ]


                # --------------------------------------------
                # PRINT RESULTS
                # --------------------------------------------

                print("\n================================")
                print("DEPTH CALIBRATION TEST")
                print("================================")

                print(
                    f"Pixel: ({x}, {y})"
                )

                print(
                    f"Valid samples: "
                    f"{stats['count']}"
                )

                print(
                    f"Median depth: "
                    f"{stats['median'] * 1000:.2f} mm"
                )

                print(
                    f"Mean depth: "
                    f"{stats['mean'] * 1000:.2f} mm"
                )

                print(
                    f"Std deviation: "
                    f"{stats['std'] * 1000:.2f} mm"
                )

                print(
                    f"Min depth: "
                    f"{stats['min'] * 1000:.2f} mm"
                )

                print(
                    f"Max depth: "
                    f"{stats['max'] * 1000:.2f} mm"
                )

                print(
                    f"Depth spread: "
                    f"{(stats['max'] - stats['min']) * 1000:.2f} mm"
                )

                print(
                    "Camera XYZ: "
                    f"[{camera_xyz_mm[0]:.2f}, "
                    f"{camera_xyz_mm[1]:.2f}, "
                    f"{camera_xyz_mm[2]:.2f}] mm"
                )


                # --------------------------------------------
                # DRAW CLICKED POINT
                # --------------------------------------------

                cv2.circle(
                    image,
                    (x, y),
                    7,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    image,
                    f"Z={camera_xyz_mm[2]:.1f}mm",
                    (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

            # Reset so one click is processed once
            clicked_point = None


        # ----------------------------------------------------
        # DISPLAY
        # ----------------------------------------------------

        cv2.putText(
            image,
            "Click marker center repeatedly | Q = quit",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2
        )

        cv2.imshow(
            "Depth Calibration Test",
            image
        )

        key = cv2.waitKey(1) & 0xFF

        if key in (
            ord("q"),
            ord("Q")
        ):
            print("\nQ pressed.")
            break


except KeyboardInterrupt:

    print(
        "\nCtrl+C pressed."
    )


finally:

    pipeline.stop()

    cv2.destroyAllWindows()

    print(
        "\nRealSense stopped."
    )