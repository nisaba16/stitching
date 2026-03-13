#!/bin/bash

# Build and push Docker image to AWS ECR
# Repository: vista/batch-stitching
# Region: eu-west-1

set -e

AWS_ACCOUNT_ID="693878105796"
AWS_REGION="eu-west-1"
AWS_PROFILE="dev"
ECR_REPO="vista/batch-stitching"
IMAGE_TAG="${1:-latest}"

export AWS_PROFILE

ECR_URI="${AWS_ACCOUNT_ID}.dkr.ecr.${AWS_REGION}.amazonaws.com"
FULL_IMAGE="${ECR_URI}/${ECR_REPO}:${IMAGE_TAG}"

echo "=== Build & Push to ECR ==="
echo "  Repo:  ${ECR_REPO}"
echo "  Tag:   ${IMAGE_TAG}"
echo "  URI:   ${FULL_IMAGE}"
echo ""

# Authenticate Docker to ECR
echo "Authenticating Docker to ECR..."
aws ecr get-login-password --region "${AWS_REGION}" | docker login --username AWS --password-stdin "${ECR_URI}"

# Build the Docker image
echo ""
echo "Building Docker image..."
docker build --platform linux/amd64 -t "${ECR_REPO}:${IMAGE_TAG}" .

# Tag for ECR
echo ""
echo "Tagging image for ECR..."
docker tag "${ECR_REPO}:${IMAGE_TAG}" "${FULL_IMAGE}"

# Push to ECR
echo ""
echo "Pushing to ECR..."
docker push "${FULL_IMAGE}"

echo ""
echo "Done! Image pushed to: ${FULL_IMAGE}"
