#!/bin/bash
set -e
# set -x

cd "$(dirname "$0")"

display_help()
{
    echo ""
    echo "This script is a wrapper for docker compose, and can execute any docker compose commands."
    echo "It makes use of .env file(s) to establish consistent configuration among deployments."
    echo "It will handle any modifications/additions that need to be done."
    echo "The default path is '$_ENV_PATH'."
    echo "Use the '-f' or '--file' prefix to set a custom path."
    echo ""
    echo "Usage: $0 [OPTIONS] COMMAND"
    echo ""
    echo "General Options:"
    echo "  --help, -h                      Display this help message."
    echo "  --more-help, -mh                Display docker compose help message."
    echo ""
    echo "File Options:"
    echo "  --file, -f <string>             Compose configurations file."
    echo "  --env-file <string>             Path of ENV."
    echo "  --template <string>             Path of template ENV."
    echo "  --rewrite                       If the ENV already exists then rewrite it."
    echo "  --nowrite                       Pass in variables temporarily, do not write to any file."
    echo "  --variable <string> <string>    Define custom environment variable with a value."
    echo ""
    echo "Deployment Options:"
    echo "  --build <path>                  Builds containers before deploying using the given external package path."
    echo "  --build-no-cache <path>         Builds containers before deploying without any build cache using the given external package path."
    echo "  --windows                       Run setup for windows server CARLA instance (for testing purposes only)."
    echo "  --simrec                        Deploys in simulation recording mode."
    echo "  --nosim                         Deploy in no-simulator mode. The scenario will be played back from a recording (Less GPU intensive)."
    echo "  --recording-name, -rn <string>  Name of competition recording."
    echo "  --recording-path, -rp <string>  Path on host machine where competition recordings are stored."
    echo ""
    echo "Simulator Mode Options:"
    echo "  --port <integer>                Hosting port number (even number recommended)."
    echo "  --custom-host, -ch <string>     Custom hostname/ip of CARLA simulator."
    echo "  --scenario                      Specify the simulation scenario to run (see documentation for options)."
    echo ""
    echo "No Simulator Mode Options:"
    echo "  --playback-name, -pn <string>   Name of scenario playback file that will be used for nosim mode."
    echo ""
    echo "Run '$0 [...] --more-help' for docker compose specific information"
}

# Creates or changes ENV variables
create_or_change_variable () 
{
    # If the env does not exist then create it.
    if [ ! -f $_ENV_PATH ]; then
        if [ ! -f $_TEMPLATE_PATH ]; then
            touch $_ENV_PATH
        else
            cp $_TEMPLATE_PATH $_ENV_PATH
        fi
    fi

    _ENV_KEY=$1
    _ENV_VAL=$2

    # This allows the env to be parsed and read
    source $_ENV_PATH

    if ! grep -q "^$_ENV_KEY=" $_ENV_PATH; then
        echo -e "\n$_ENV_KEY=$_ENV_VAL" >> $_ENV_PATH
    else
        sed -i "s/^$_ENV_KEY=.*/$_ENV_KEY=$_ENV_VAL/" $_ENV_PATH
    fi
}

# Determines if the passed in argument has the correct number of parameters
# is_arg_valid <arg> <n> <param-1> <param-2> ... <param-n>
is_arg_valid ()
{
    local _ARGUMENT=$1
    local _NUMBER_OF_EXPECTED_PARAMS=$2
    shift 2

    case $_NUMBER_OF_EXPECTED_PARAMS in
        1)
            if [[ -z $1 ]]; then
                echo "Argument '${_ARGUMENT}' requires ${_NUMBER_OF_EXPECTED_PARAMS} parameters."
                exit 0
            fi
        ;;

        2)
            if [[ -z $1 || -z $2 ]]; then
                echo "Argument '${_ARGUMENT}' requires ${_NUMBER_OF_EXPECTED_PARAMS} parameters."
                exit 0
            fi
        ;;
    esac
}

# Declare variables
declare -A _VAR_DICT
_VAR_DICT["DTCVC_PLATFORM"]="linux"
_VAR_DICT["DTCVC_NO_SIM_MODE"]=false

_NO_WRITE=false
_REWRITE=false
_BUILD=false
_NO_CACHE=false
_BASE_DEPLOYMENT_PATH="./dtcvc/deployment"
_BASE_OUTPUT_PATH="./output"
_COMP_RECORDING_PATH="$_BASE_OUTPUT_PATH/recordings"
_SCENARIO_RECORDING_PATH="$_BASE_OUTPUT_PATH/scenario_recordings"
_COMPOSE_FILE_PATH="$_BASE_DEPLOYMENT_PATH/dtcvc-deployment.yml"
_ENV_PATH="$_BASE_DEPLOYMENT_PATH/config/.env"
_TEMPLATE_PATH="$_BASE_DEPLOYMENT_PATH/config/.env.example"
_DOCKER_COMPOSE_ARGS=()

# No arguments
if [[ $# -eq 0 ]]; then
    display_help
    exit 0
fi

# Get options for argument parsing
# "is_arg_valid" is only necessary for options with more than one argument
# Format:
# <option>)
#   is_arg_valid $_curr_arg <n> $1 $2 ... $<n>
#   <operation>
#   shift <n>
# ;;
while (( "$#" )); do
    _curr_arg=$1
    shift
    case $_curr_arg in
        "-h"|"--help")
            display_help
            exit 0
        ;;

        "-mh"|"--more-help")
            _DOCKER_COMPOSE_ARGS+=("--help")
        ;;

        "--template")
            is_arg_valid $_curr_arg 1 $1
            _TEMPLATE_PATH=$1
            shift
        ;;

        "--env-file")
            is_arg_valid $_curr_arg 1 $1
            _ENV_PATH=$1
            shift
        ;;

        "-f"|"--file")
            is_arg_valid $_curr_arg 1 $1
            _COMPOSE_FILE_PATH=$1
            shift
        ;;

        "--rewrite")
            _REWRITE=true
        ;;

        "--nowrite")
            _NO_WRITE=true
        ;;

        "--variable")
            is_arg_valid $_curr_arg 2 $1 $2
            _VAR_DICT["$1"]=$2
            shift 2
        ;;

        "--build")
            _BUILD=true
            _BUILD_EXT_PATH=$1
            shift
        ;;

        "--build-no-cache")
            _BUILD=true
            _NO_CACHE=true
            _BUILD_EXT_PATH=$1
            shift
        ;;

        "--windows")
            _VAR_DICT["DTCVC_PLATFORM"]="windows"
        ;;

        "--simrec")
            _VAR_DICT["DTCVC_SIM_REC_MODE"]=true
            _DOCKER_COMPOSE_SERVICES+=("simulator" "scorekeeper")
        ;;

        "--nosim")
            _VAR_DICT["DTCVC_NO_SIM_MODE"]=true
            _COMPOSE_FILE_PATH="$_BASE_DEPLOYMENT_PATH/dtcvc-deployment.nosim.yml"
        ;;

        "--scenario")
            _VAR_DICT["DTCVC_SIM_SCENARIO"]=$(realpath $1)
            shift
        ;;

        "--port")
            is_arg_valid $_curr_arg 1 $1
            _VAR_DICT["DTCVC_SIM_PORT"]=$1
            _VAR_DICT["DTCVC_SIM_PORT_RANGE"]=$(($1 + 2))
            shift
        ;;

        "-ch"|"--custom-host")
            is_arg_valid $_curr_arg 1 $1
            _VAR_DICT["DTCVC_SIM_CUSTOM_HOSTNAME"]=$1
            shift
        ;;

        "-pn"|"--playback-name")
            is_arg_valid $_curr_arg 1 $1
            _VAR_DICT["DTCVC_NO_SIM_PLAYBACK_FILE_NAME"]=$1
            shift
        ;;

        "-rn"|"--recording-name")
            is_arg_valid $_curr_arg 1 $1
            _VAR_DICT["DTCVC_RECORDING_FILE_NAME"]=$1
            shift
        ;;

        "-rp"|"--recording-path")
            is_arg_valid $_curr_arg 1 $1
            _VAR_DICT["DTCVC_SCENARIO_RECORDINGS_PATH"]=$1
            shift
        ;;

        *)
            _DOCKER_COMPOSE_ARGS+=("$_curr_arg")
        ;;
    esac
done

# Add simulator profile
if [[ "${_VAR_DICT["DTCVC_PLATFORM"]}" == "linux" && "${_VAR_DICT["DTCVC_NO_SIM_MODE"]}" == false ]]; then
    _PROFILES+="--profile simulator"
fi

_VAR_DICT["DTCVC_WAIT_FOR_SIM"]=!${_VAR_DICT["DTCVC_NO_SIM_MODE"]}

# Build before deploying
if [ "$_BUILD" == true ]; then
    if [ "$_NO_CACHE" == true ]; then
        ./build.sh $_BUILD_EXT_PATH --no-cache
    else
        ./build.sh $_BUILD_EXT_PATH
    fi
fi

_TEMP_ENV_VARS=""
# If ENV file exists
if [ -f $_ENV_PATH ]; then
    if [[ "$_NO_WRITE" != false && "$_REWRITE" != true ]]; then
        echo "NOTICE: ENV modification disabled"
        echo "NOTICE: Please activate 'rewrite mode' with the '--rewrite' flag to write to the ENV"
    fi

    for key in "${!_VAR_DICT[@]}"; do
        value="${_VAR_DICT[$key]}"

        # Only write to file if in rewrite mode alone
        if [[ "$_NO_WRITE" == false && "$_REWRITE" == true ]]; then
            create_or_change_variable $key $value
        else
            _TEMP_ENV_VARS+="${key}=${value} "
        fi
    done
else
    if [[ "$_NO_WRITE" == true && "$_REWRITE" == false ]]; then
        echo "NOTICE: ENV modification disabled"
        echo "NOTICE: Please activate 'rewrite mode' with the '--rewrite' flag to write to the ENV"
    fi

    for key in "${!_VAR_DICT[@]}"; do
        value="${_VAR_DICT[$key]}"

        # Only write to file if not in no write mode alone
        if [[ "$_NO_WRITE" == true && "$_REWRITE" == false ]]; then
            _TEMP_ENV_VARS+="${key}=${value} "
        else
            create_or_change_variable $key $value
        fi
    done
fi

# Run docker compose command
source $_ENV_PATH

# Change simulator requirement
# if [[ "${_VAR_DICT["DTCVC_PLATFORM"]}" == "windows" || "${DTCVC_NO_SIM_MODE,,}" == "true" || "${_VAR_DICT["DTCVC_NO_SIM_MODE"]}" == true ]]; then
#     sed -i -e "/simulator:/{n;s/required: .*/required: false/}" "$_COMPOSE_FILE_PATH"
# else
#     sed -i -e "/simulator:/{n;s/required: .*/required: true/}" "$_COMPOSE_FILE_PATH"
# fi

# Create comp recording path
if [[ ! -d "$_COMP_RECORDING_PATH" ]]; then
    mkdir $_COMP_RECORDING_PATH
fi

# Create scenario recording path
if [[ ! -d "$_SCENARIO_RECORDING_PATH" ]]; then
    mkdir $_SCENARIO_RECORDING_PATH
fi


_DOCKER_COMPOSE_ARGS+=("--abort-on-container-exit")

set -x
env $_TEMP_ENV_VARS docker compose --env-file $_ENV_PATH --file $_COMPOSE_FILE_PATH $_PROFILES "${_DOCKER_COMPOSE_ARGS[@]}" "${_DOCKER_COMPOSE_SERVICES[@]}"
