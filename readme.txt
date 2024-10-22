0. 참고자료
  가. https://docs.ultralytics.com/ko/guides/ros-quickstart/#using-yolo-with-depth-images

1. usb cam
  가. 설치(필요시)
    1) sudo apt install ros-noetic-usb-cam
  나. 픽셀 포맷 확인
    1) $ v4l2-ctl --list-formats
    2) $ v4l2-ctl --list-formats-ext