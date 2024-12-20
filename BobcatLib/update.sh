#!/bin/bash
./gradlew :spotlessApply
./gradlew publish
cp -rf build/repos/releases/BobcatLib BobcatLib/repos/
cp -rf build/repos/releases/BobcatLib ../gh-pages/BobcatLib/repos/
