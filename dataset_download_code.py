# ------------- DRONE DATASET CODE-------------------#
!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="gHG4MB4tXUkQNPiVd6Nt")
project = rf.workspace("1-d7rzc").project("drone-dataset-8rjn2")
version = project.version(1)
dataset = version.download("yolov11")


# ------------- BIRDS DATASET CODE-------------------#

!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="gHG4MB4tXUkQNPiVd6Nt")
project = rf.workspace("detectiondanimaux").project("birds-detect")
version = project.version(16)
dataset = version.download("yolov11")


# ------------- MILITARY_AIRCRAFT DATASET CODE-------------------#

!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="gHG4MB4tXUkQNPiVd6Nt")
project = rf.workspace("new-workspace-0k81p").project("military-aircraft-yl7jf")
version = project.version(1)
dataset = version.download("yolov11")

# ------------- COMMERCIAL_AEROPLANE DATASET CODE-------------------#

!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="gHG4MB4tXUkQNPiVd6Nt")
project = rf.workspace("shimaa").project("plane-g1ief")
version = project.version(1)
dataset = version.download("yolov11")
