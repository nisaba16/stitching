#!/bin/bash

# Script de test pour le système de stitching Docker

set -e

echo "=== Test du système de stitching Docker ==="
echo ""

# Vérifier que Docker est installé
if ! command -v docker &> /dev/null; then
    echo "❌ Docker n'est pas installé"
    exit 1
fi

echo "✅ Docker détecté"

# Builder l'image
echo ""
echo "📦 Construction de l'image Docker..."
docker build -t football-stitching:latest .

if [ $? -eq 0 ]; then
    echo "✅ Image Docker construite avec succès"
else
    echo "❌ Échec de la construction Docker"
    exit 1
fi

# Vérifier que l'exécutable est présent dans l'image
echo ""
echo "🔍 Vérification de l'exécutable dans l'image..."
docker run --rm football-stitching:latest ls -la /app/build/image-stitching

# Test de base (help)
echo ""
echo "🧪 Test de base..."
docker run --rm football-stitching:latest

echo ""
echo "✅ Tests terminés avec succès!"
echo ""
echo "🚀 Votre image Docker est prête:"
echo "   - Image: football-stitching:latest"
echo "   - Upload S3: football-stitched-videos"
echo "   - Format: match_{uuid}/output.mp4"
echo ""
echo "📖 Consultez DEPLOYMENT_GUIDE.md pour les instructions d'utilisation"