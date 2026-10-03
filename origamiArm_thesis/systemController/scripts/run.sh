printf "\nThis script will now attempt to run systemcontroller (alternatively you
        can install the module).\n\n Ensure that the shared kinematics library has
        been compiled FOR THIS ARCHITECTURE (see script in kinematicsExternalLib folder).\n"
printf "Changing to main module directory...\n"
cd ../src/systemcontroller/
printf "Running...\n"
python3 main.py

