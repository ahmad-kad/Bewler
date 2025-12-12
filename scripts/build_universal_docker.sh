#!/bin/bash

# Universal Docker Build Script for URC 2026
# Builds all services or individual services as needed

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Available services
SERVICES=(
    "state-management-service"
    "safety-service"
    "navigation-service"
    "vision-service"
    "led-service"
    "sensor-bridge-service"
    "slam-service"
    "mission-service"
    "websocket-bridge-service"
    "can-mock-service"
    "frontend-service"
    "frontend-prod"
    "development"
    "help"
)

# Default values
BUILD_TYPE="release"
SERVICES_TO_BUILD=()
PARALLEL_BUILD=true
PUSH_TO_REGISTRY=false
REGISTRY_PREFIX="urc2026"

# Function to print usage
usage() {
    echo "Universal Docker Build Script for URC 2026 Robotics Platform"
    echo ""
    echo "Usage: $0 [OPTIONS] [SERVICES...]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  -d, --debug            Build in debug mode (development)"
    echo "  -r, --release          Build in release mode (production) [default]"
    echo "  -s, --serial           Build services serially (not in parallel)"
    echo "  -p, --push             Push images to registry after building"
    echo "  --registry PREFIX      Registry prefix (default: urc2026)"
    echo "  --all                  Build all services"
    echo ""
    echo "Available Services:"
    for service in "${SERVICES[@]}"; do
        echo "  - $service"
    done
    echo ""
    echo "Examples:"
    echo "  $0 --all                    # Build all services in parallel"
    echo "  $0 safety-service navigation-service  # Build specific services"
    echo "  $0 --debug --serial state-management-service  # Debug build single service"
    echo "  $0 --push --registry myregistry.com/urc2026 frontend-prod  # Push to registry"
}

# Function to validate service name
validate_service() {
    local service="$1"
    for valid_service in "${SERVICES[@]}"; do
        if [[ "$service" == "$valid_service" ]]; then
            return 0
        fi
    done
    return 1
}

# Function to build a service
build_service() {
    local service="$1"
    local tag="${REGISTRY_PREFIX}:${service}"

    echo -e "${BLUE}Building service: ${service}${NC}"

    # Build arguments
    local build_args=(
        "--build-arg" "BUILDKIT_INLINE_CACHE=1"
        "--build-arg" "BUILD_DATE=$(date -u +'%Y-%m-%dT%H:%M:%SZ')"
        "--build-arg" "GIT_COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo 'unknown')"
    )

    if [[ "$BUILD_TYPE" == "debug" ]]; then
        build_args+=("--build-arg" "BUILD_TYPE=debug")
    fi

    # Build command
    local build_cmd=(
        docker build
        "${build_args[@]}"
        --target "$service"
        --tag "$tag"
        --file "$PROJECT_ROOT/docker/Dockerfile.universal"
        "$PROJECT_ROOT"
    )

    echo -e "${YELLOW}Command: ${build_cmd[*]}${NC}"

    if "${build_cmd[@]}"; then
        echo -e "${GREEN}✅ Successfully built: $service${NC}"

        # Push to registry if requested
        if [[ "$PUSH_TO_REGISTRY" == true ]]; then
            echo -e "${BLUE}Pushing to registry: $tag${NC}"
            if docker push "$tag"; then
                echo -e "${GREEN}✅ Successfully pushed: $tag${NC}"
            else
                echo -e "${RED}❌ Failed to push: $tag${NC}"
                return 1
            fi
        fi

        return 0
    else
        echo -e "${RED}❌ Failed to build: $service${NC}"
        return 1
    fi
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            exit 0
            ;;
        -d|--debug)
            BUILD_TYPE="debug"
            shift
            ;;
        -r|--release)
            BUILD_TYPE="release"
            shift
            ;;
        -s|--serial)
            PARALLEL_BUILD=false
            shift
            ;;
        -p|--push)
            PUSH_TO_REGISTRY=true
            shift
            ;;
        --registry)
            REGISTRY_PREFIX="$2"
            shift 2
            ;;
        --all)
            SERVICES_TO_BUILD=("${SERVICES[@]}")
            shift
            ;;
        *)
            # Check if it's a valid service name
            if validate_service "$1"; then
                SERVICES_TO_BUILD+=("$1")
            else
                echo -e "${RED}Error: Unknown service '$1'${NC}"
                echo ""
                usage
                exit 1
            fi
            shift
            ;;
    esac
done

# If no services specified, show usage
if [[ ${#SERVICES_TO_BUILD[@]} -eq 0 ]]; then
    echo -e "${RED}Error: No services specified${NC}"
    echo ""
    usage
    exit 1
fi

# Print build configuration
echo -e "${BLUE}URC 2026 Universal Docker Build${NC}"
echo -e "${BLUE}=================================${NC}"
echo -e "Build Type: ${BUILD_TYPE}"
echo -e "Parallel Build: ${PARALLEL_BUILD}"
echo -e "Push to Registry: ${PUSH_TO_REGISTRY}"
if [[ "$PUSH_TO_REGISTRY" == true ]]; then
    echo -e "Registry Prefix: ${REGISTRY_PREFIX}"
fi
echo -e "Services to Build: ${SERVICES_TO_BUILD[*]}"
echo ""

# Enable BuildKit for better performance
export DOCKER_BUILDKIT=1

# Build services
if [[ "$PARALLEL_BUILD" == true && ${#SERVICES_TO_BUILD[@]} -gt 1 ]]; then
    echo -e "${YELLOW}Building services in parallel...${NC}"

    # Build services in parallel
    pids=()
    failed_services=()

    for service in "${SERVICES_TO_BUILD[@]}"; do
        build_service "$service" &
        pids+=($!)
    done

    # Wait for all builds to complete
    for i in "${!pids[@]}"; do
        if ! wait "${pids[$i]}"; then
            failed_services+=("${SERVICES_TO_BUILD[$i]}")
        fi
    done

    # Report results
    if [[ ${#failed_services[@]} -gt 0 ]]; then
        echo -e "${RED}❌ Build failures: ${failed_services[*]}${NC}"
        exit 1
    else
        echo -e "${GREEN}✅ All services built successfully!${NC}"
    fi

else
    echo -e "${YELLOW}Building services serially...${NC}"

    # Build services serially
    failed_services=()

    for service in "${SERVICES_TO_BUILD[@]}"; do
        if ! build_service "$service"; then
            failed_services+=("$service")
        fi
    done

    # Report results
    if [[ ${#failed_services[@]} -gt 0 ]]; then
        echo -e "${RED}❌ Build failures: ${failed_services[*]}${NC}"
        exit 1
    else
        echo -e "${GREEN}✅ All services built successfully!${NC}"
    fi
fi

# Print next steps
echo ""
echo -e "${BLUE}Next Steps:${NC}"
echo "1. Start development environment:"
echo -e "   ${YELLOW}docker-compose -f docker-compose.universal.yml up${NC}"
echo ""
echo "2. Start production environment:"
echo -e "   ${YELLOW}docker-compose -f docker-compose.prod.yml up${NC}"
echo ""
echo "3. View available services:"
echo -e "   ${YELLOW}docker images | grep urc2026${NC}"
