import sys
from pathlib import Path

import cv2
import numpy as np
import torch


FYP_DIR = Path.home() / "FYP"
MIDAS_DIR = FYP_DIR / "MiDaS"
WEIGHTS = MIDAS_DIR / "weights" / "dpt_hybrid_384.pt"

sys.path.insert(0, str(MIDAS_DIR))

import midas.transforms as transforms


class MiDaSProcessor:

    def __init__(self):
        print("Loading MiDaS...")

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        print("Device:", self.device)

        self.model = torch.hub.load(
            str(MIDAS_DIR),
            "DPT_Hybrid",
            source="local",
            pretrained=False
        )

        checkpoint = torch.load(
            str(WEIGHTS),
            map_location=self.device,
            weights_only=False
        )

        if "model" in checkpoint:
            checkpoint = checkpoint["model"]

        self.model.load_state_dict(checkpoint, strict=False)
        self.model.to(self.device)
        self.model.eval()

        self.transform = lambda sample: transforms.PrepareForNet()(
    transforms.NormalizeImage(
        mean=np.array([0.485, 0.456, 0.406]),
        std=np.array([0.229, 0.224, 0.225])
    )(
        transforms.Resize(
            384, 384,
            resize_target=False,
            keep_aspect_ratio=True,
            ensure_multiple_of=32,
            resize_method="upper_bound",
            image_interpolation_method=3
        )(sample)
    )
)

        print("MiDaS loaded successfully.")
        print("GPU:", torch.cuda.get_device_name(0)
              if torch.cuda.is_available() else "CPU")

    def predict(self, image_bgr):

        image_rgb = cv2.cvtColor(
            image_bgr,
            cv2.COLOR_BGR2RGB
        )

        input_batch = self.transform({"image": image_rgb})["image"]
        input_batch = torch.from_numpy(input_batch)

        if input_batch.ndim == 3:
            input_batch = input_batch.unsqueeze(0)

        input_batch = input_batch.to(self.device)

        with torch.no_grad():
            prediction = self.model(input_batch)

            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=image_rgb.shape[:2],
                mode="bicubic",
                align_corners=False
            ).squeeze()

        depth = prediction.cpu().numpy()

        depth = np.nan_to_num(
            depth,
            nan=0.0,
            posinf=0.0,
            neginf=0.0
        )

        return depth


if __name__ == "__main__":

    processor = MiDaSProcessor()

    input_dir = FYP_DIR / "video_frames"

    images = sorted(
        list(input_dir.glob("*.jpg")) +
        list(input_dir.glob("*.png"))
    )

    if not images:
        print("ERROR: No test image found in:")
        print(input_dir)
        sys.exit(1)

    image_path = images[0]

    print()
    print("Test image:", image_path)

    image = cv2.imread(str(image_path))

    if image is None:
        print("ERROR: Could not read image.")
        sys.exit(1)

    depth = processor.predict(image)

    output_dir = FYP_DIR / "backend" / "depth" / "output"
    output_dir.mkdir(parents=True, exist_ok=True)

    depth_n = cv2.normalize(
        depth,
        None,
        0,
        255,
        cv2.NORM_MINMAX
    ).astype(np.uint8)

    output_file = output_dir / "test_depth.png"

    cv2.imwrite(
        str(output_file),
        depth_n
    )

    print()
    print("Depth shape:", depth.shape)
    print("Depth min:", float(depth.min()))
    print("Depth max:", float(depth.max()))
    print("Output:", output_file)
    print()
    print("MIDAS TEST SUCCESS")
