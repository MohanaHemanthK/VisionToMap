# VisionToMap: Real-Time Object Localization and Mapping

**VisionToMap** is a Python-based project that maps real-time object tracking from a live camera feed onto a pre-defined map layout. It leverages computer vision and depth sensing to dynamically track and visualize object positions within a defined environment.

---

## Features
- **Real-Time Object Tracking**: Tracks objects using YOLOv8 and maps their location based on depth data.
- **Pre-Defined Map Integration**: Displays the tracked object's position on a custom map layout.
- **Depth and Angle Calculation**: Uses Intel Realsense depth data to estimate distances and angles for accurate localization.
- **Coordinate Finder**: Utility to mark key reference points on the map or video feed.

---

## Requirements
To run this project, ensure you have the following dependencies installed:

- **Python** (>=3.7)
- **OpenCV**: For image and video processing.
- **Intel Realsense SDK**: For depth sensing.
- **Ultralytics YOLO**: For object detection.
- **NumPy**: For numerical computations.

Install dependencies using:
```bash
pip install opencv-python pyrealsense2 ultralytics numpy
```

---

## How to Run
1. Clone the repository:
    ```bash
    git clone https://github.com/MohanaHemanthK/VisionToMap.git
    cd VisionToMap
    ```

2. Prepare your map image:
    - Place your map layout image in the project directory.
    - Update the `Map` variable in `ObjectMapping.py` with the path to your map file.

3. Start the mapping program:
    ```bash
    python ObjectMapping.py
    ```

4. Optionally, use the `CoordinateFinder.py` to find reference coordinates for your map:
    ```bash
    python CoordinateFinder.py
    ```

---

## Project Structure
```
VisionToMap/
├── ObjectMapping.py      # Main script for object tracking and mapping
├── CoordinateFinder.py   # Tool for finding reference coordinates
├── map.jpg               # Example map layout (replace with your own)
├── requirements.txt      # Project dependencies
└── README.md             # Project documentation
```

---

## Usage
1. Launch `ObjectMapping.py` to track objects in real-time.
2. Visualize the object's position dynamically updated on the map.
3. Use `CoordinateFinder.py` to set or verify reference points on your map or video feed.

---

## Future Improvements
- Multi-object tracking support.
- Enhanced accuracy using advanced tracking algorithms.
- 3D mapping for improved visualization.
- Integration with IoT and robotics platforms.
- I will integrate the sterolab's ZED camera to capture more acurately because ZED 2i camera has higher depth sensing range (meaning it can estimate distance acurately upto 30 meters)
