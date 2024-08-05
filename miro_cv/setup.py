# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD!
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# Fetch values from package.xml.
setup_args = generate_distutils_setup(
    packages=["miro_cv" "miro_cv.nodes" "miro_cv.utils"], scripts=["scripts/start_mdk.sh"],package_dir={"": "src"},#"nodes" "nodes.utils"
)
setup(**setup_args)
