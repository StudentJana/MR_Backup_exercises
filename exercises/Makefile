build: build/01/ChangeTheWorld.pdf build/02/Wanderer.pdf build/03/SensorModel.pdf build/04/MotionModel.pdf build/05/EKF-Selflocalization.pdf build/06/goto.pdf build/setup/Install.pdf

clean:
	rm -rf build

build/01/ChangeTheWorld.pdf: 01/ChangeTheWorld.md
	mkdir -p build/01
	pandoc 01/ChangeTheWorld.md -o build/01/ChangeTheWorld.pdf --resource-path 01

build/02/Wanderer.pdf: 02/Wanderer.md
	mkdir -p build/02
	pandoc 02/Wanderer.md -o build/02/Wanderer.pdf --resource-path 02

build/03/SensorModel.pdf: 03/SensorModel.md
	mkdir -p build/03
	pandoc 03/SensorModel.md -o build/03/SensorModel.pdf --resource-path 03

build/04/MotionModel.pdf: 04/MotionModel.md
	mkdir -p build/04
	pandoc 04/MotionModel.md -o build/04/MotionModel.pdf --resource-path 04

build/05/EKF-Selflocalization.pdf: 05/EKF-Selflocalization.md
	mkdir -p build/05
	pandoc 05/EKF-Selflocalization.md -o build/05/EKF-Selflocalization.pdf --resource-path 05

build/06/goto.pdf: 06/goto.md
	mkdir -p build/06
	pandoc 06/goto.md -o build/06/goto.pdf --resource-path 06

build/setup/Install.pdf: setup/Install.md
	mkdir -p build/setup
	pandoc setup/Install.md -o build/setup/Install.pdf --resource-path setup
