#!/bin/bash

source "/setup.bash"

# Run exec command directly
if [ "$1" == "exec" ]; then
    $@
else
    if [[ -z "${PACKAGE_NAME}" ]]; then
        echo "Environment Variable 'PACKAGE_NAME' not defined"
        echo "Stopping container..."
    else
        # e.g. 'dtcvc_scorekeeper'
        _PACKAGE_NAME="${PACKAGE_NAME}"

        _EXTRA_PARAMETERS=$@

        _LAUNCH_NAME=""
        _HOST=""
        _PORT=""
        _NO_SIM_PLAYBACK_FILE_NAME=""
        _RECORDING_FILE_NAME=""

        # Determine if to run scenario-by-carla or scenario-by-rosbag
        if [[ -z "${DTCVC_NO_SIM_MODE}" || "${DTCVC_NO_SIM_MODE,,}" == "false" ]]; then # scenario-by-carla

            # Determine where to enter simulation recording mode
            if [[ -n "${DTCVC_SIM_REC_MODE}" && "${DTCVC_SIM_REC_MODE,,}" == "true" ]]; then
                _LAUNCH_NAME="${_PACKAGE_NAME}_sim_rec.launch.py"
            elif [[ -n "${DTCVC_SIM_ONLY_MODE}" && "${DTCVC_SIM_ONLY_MODE,,}" == "true" ]]; then
                _LAUNCH_NAME="${_PACKAGE_NAME}_sim_only.launch.py"
            else
                _LAUNCH_NAME="${_PACKAGE_NAME}.launch.py"
            fi


            # # Check if custom hostname passed in
            # if [[ -n "${DTCVC_SIM_CUSTOM_HOSTNAME}" && "${DTCVC_SIM_CUSTOM_HOSTNAME}" != "" ]]; then
            #     echo "WARNING: environment variable CUSTOM_HOSTNAME set"
            #     echo "Please ensure that this hostname is valid as it overrides the default hosts"
            #     echo ""
            #     _HOST="host:=${DTCVC_SIM_CUSTOM_HOSTNAME}"
            # else
            #     # Set host for windows if not developing for linux
            #     if [[ -n "${DTCVC_PLATFORM}" && "${DTCVC_PLATFORM,,}" == "windows" ]]; then
            #         echo "NOTICE: Due to limitations with CARLA for Windows you must bring up the windows instance of CARLA on your desktop and not within a container."
            #         echo "NOTICE: CARLA for Linux is not compatible with WSL."
            #         echo "NOTICE: Running CARLA server on windows is NOT recommended for submissions, it is purely for testing."
            #         echo ""
            #         _HOST="host:=host.docker.internal"
            #     else
            #         _HOST="host:=dtcvc-simulator"
            #     fi

            #     # Set port for sim
            #     if [[ -n "${DTCVC_SIM_PORT}" && "${DTCVC_SIM_PORT}" != "" ]]; then
            #         _PORT="port:=${DTCVC_SIM_PORT}"
            #     else
            #         _PORT="port:=2000"
            #     fi
            # fi
        else # scenario-by-rosbag
            _LAUNCH_NAME="${_PACKAGE_NAME}_no_sim.launch.py"

            # Handle no sim playback file path
            if [[ -n "${DTCVC_NO_SIM_PLAYBACK_FILE_NAME}" && "${DTCVC_NO_SIM_PLAYBACK_FILE_NAME}" != "" ]]; then
                _NO_SIM_PLAYBACK_FILE_NAME="playback_file_name:=${DTCVC_NO_SIM_PLAYBACK_FILE_NAME}"
            fi
        fi

        # Handle comp. recording file path
        if [[ -n "${DTCVC_RECORDING_FILE_NAME}" && "${DTCVC_RECORDING_FILE_NAME}" != "" ]]; then
            _RECORDING_FILE_NAME="recording_file_name:=${DTCVC_RECORDING_FILE_NAME}"
        fi

        # Define command
        set -x
        exec ros2 launch $_PACKAGE_NAME $_LAUNCH_NAME $_HOST $_PORT $_NO_SIM_PLAYBACK_FILE_NAME $_RECORDING_FILE_NAME $_EXTRA_PARAMETERS
        set +x
    fi
fi