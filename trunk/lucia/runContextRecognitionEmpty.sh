#!/bin/sh
rm -rf ~/.ros/domains
mkdir ~/.ros/domains
cp domains/contextInferenceEmptyExample.ddl ~/.ros/domains/currentContextInferenceDomain.ddl
./contextrecognition/build/install/contextrecognition/bin/contextrecognition se.oru.lucia.contextrecognition.ContextRecognitionLucia
