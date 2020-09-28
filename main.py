from Interface.InterfaceForStudent import L515_basic_interface


camera = L515_basic_interface()
camera.startStream()
# camera.liveVideo()
# camera.savePicture()
camera.livePicture()
camera.stopStream()