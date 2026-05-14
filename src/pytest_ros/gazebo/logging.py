class _VideoRecorder:
    """
    Subscribes to a ROS image topic and writes frames to an MP4 file.

    Usage::

        recorder = _VideoRecorder(node, CAMERA_TOPIC, "/tmp/sim_logs/run.mp4")
        recorder.start()
        # ... run test ...
        recorder.stop()   # flushes and closes the file
    """

    def __init__(self, node, topic: str, output_path: str, fps: int = VIDEO_FPS):
        self._node = node
        self._output_path = output_path
        self._fps = fps
        self._writer: cv2.VideoWriter | None = None
        self._lock = threading.Lock()
        self._sub = None
        self._topic = topic

    def start(self) -> None:
        self._sub = self._node.create_subscription(
            Image,
            self._topic,
            self._on_image,
            10,
        )
        logger.info(f"[VIDEO] Recording {self._topic} → {self._output_path}")

    def stop(self) -> None:
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None
        with self._lock:
            if self._writer is not None:
                self._writer.release()
                self._writer = None
                logger.info(f"[VIDEO] Saved {self._output_path}")

    def _on_image(self, msg: Image) -> None:
        """Convert a sensor_msgs/Image to BGR and append to the video file."""
        try:
            frame = self._decode(msg)
        except Exception as exc:
            logger.warning(f"[VIDEO] Frame decode error: {exc}")
            return

        with self._lock:
            if self._writer is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self._writer = cv2.VideoWriter(
                    self._output_path, fourcc, self._fps, (w, h)
                )
            self._writer.write(frame)

    @staticmethod
    def _decode(msg: Image) -> np.ndarray:
        """Return a BGR uint8 ndarray from a sensor_msgs/Image."""
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ("rgb8", "bgr8", "mono8"):
            channels = 1 if enc == "mono8" else 3
            frame = data.reshape((msg.height, msg.width, channels))
            if enc == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif enc == "mono8":
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif enc == "rgba8":
            frame = data.reshape((msg.height, msg.width, 4))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        elif enc in ("16uc1", "mono16"):
            data16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                (msg.height, msg.width)
            )
            frame = cv2.cvtColor(
                (data16 >> 8).astype(np.uint8), cv2.COLOR_GRAY2BGR
            )
        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

        return frame
