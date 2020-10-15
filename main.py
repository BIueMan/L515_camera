from Interface.InterfaceForStudent import L515_basic_interface


camera = L515_basic_interface()
camera.startStream()
# camera.liveVideo()
# camera.savePicture()
camera.livePicture()
# color, depth = camera.loadPicture("color_10,15,20_11;32;22.png", "depth_10,15,20_11;32;22.png")
camera.stopStream()