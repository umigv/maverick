import cv2

def list_available_cameras(max_index_to_check=10):
    """
    Checks for available camera indices by attempting to open each one.

    Returns:
    A list of indices for available cameras.
    """
    available_cameras = []
    for i in range(max_index_to_check):
        # Try to open the camera with the current index
        cap = cv2.VideoCapture(i)

        # Check if the camera was opened successfully
        if cap is not None and cap.isOpened():
            print(f"Camera index {i:02d} is available.")
            available_cameras.append(i)
            # You can add platform-specific backend flags here if needed (e.g., cv2.CAP_DSHOW for Windows)

            # Release the camera to make it available for other processes
            cap.release()
        else:
            print(f"Camera index {i:02d} is not available.")

    return available_cameras

if __name__ == '__main__':
    cameras = list_available_cameras()
    print(f"\nFound cameras at indices: {cameras}")