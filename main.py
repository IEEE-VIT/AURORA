import cv2
import argparse
import supervision as sv
from ultralytics import YOLO
import numpy as np
import json
import time
import paho.mqtt.client as mqtt
import ssl
import os

MQTT_BROKER = os.getenv("MQTT_BROKER")
MQTT_PORT = int(os.getenv("MQTT_PORT", "8883"))
MQTT_USER = os.getenv("MQTT_USERNAME")
MQTT_PASS = os.getenv("MQTT_PASSWORD")

ZONE_POLYGON = np.array([
    [0, 0],
    [0.5, 0],
    [0.5, 1],
    [0, 1]
])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 Live")
    parser.add_argument("--webcam-resolution", default=[1280, 720], nargs=2, type=int)
    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    frame_width, frame_height = args.webcam_resolution

    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    model = YOLO("yolov8s.pt")

    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
    mqtt_client.tls_set(
        tls_version=ssl.PROTOCOL_TLS_CLIENT
    )
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()

    last_count = -1

    box_annotator = sv.BoxAnnotator(thickness=2)
    label_annotator = sv.LabelAnnotator(text_thickness=3)

    zone_polygon = ZONE_POLYGON * np.array(args.webcam_resolution).astype(int)
    zone = sv.PolygonZone(polygon=zone_polygon)
    zone_annotator = sv.PolygonZoneAnnotator(zone=zone)

    tracker = sv.ByteTrack(
        track_activation_threshold=0.25,
        lost_track_buffer=30,
        minimum_matching_threshold=0.8
    )

    while True:
        ret, frame = cap.read()

        result = model(frame, conf=0.25, agnostic_nms=True)[0]
        detections = sv.Detections.from_ultralytics(result)
        detections = tracker.update_with_detections(detections)
        # only people
        detections = detections[detections.class_id == 0]

        labels = [
            f"person {conf:.2f}"
            for conf in detections.confidence
        ]

        frame = box_annotator.annotate(scene=frame, detections=detections)
        frame = label_annotator.annotate(
            scene=frame,
            detections=detections,
            labels=labels
        )

        # zone.trigger(detections=detections)
        inside_zone_mask = zone.trigger(detections=detections)

        detections_in_zone = detections[inside_zone_mask]
        ids_in_zone = detections_in_zone.tracker_id
        ids_in_zone = ids_in_zone[ids_in_zone != None]

        count = len(set(ids_in_zone))
        print(count)
        frame = zone_annotator.annotate(scene=frame)

        print("Publishing count:", count)
        if count != last_count:
            payload = {
                "count": count,
                "timestamp": int(time.time())
            }
            mqtt_client.publish(
                "vision/zone1",
                json.dumps(payload),
                qos=1,
                retain=True
            )
            last_count = count

        cv2.imshow("yolov8s", frame)

        if cv2.waitKey(30) == 27:
            break


if __name__ == "__main__":
    main()
