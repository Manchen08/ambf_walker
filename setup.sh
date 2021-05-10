echo "export AMBF_WALKER=$(pwd)" >> ~/.bashrc
sudo apt install libeigen3-dev
#sudo apt install ros-melodic-smach
pip3 install git+https://github.com/WPI-AIM/AIM_GaitAnalysisToolkit.git

# # shellcheck disable=SC2164
# cd lib
# git clone https://github.com/WPI-AIM-Hybrid-Exoskeleton/ilqr.git
# cd ilqr && python3 setup.py install
# cd ..

# shellcheck disable=SC2164
git clone https://github.com/WPI-AIM-Hybrid-Exoskeleton/rbdl-orb.git
# shellcheck disable=SC2164
cd rbdl-orb && sh setup.sh


