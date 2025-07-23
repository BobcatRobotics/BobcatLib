#!/bin/bash
./gradlew installRoboRIOToolchain
./gradlew :spotlessApply
./gradlew publish
cp -rf BobcatLib/repos/BobcatLib/BobcatLib-java ~/wpilib/2025/maven/BobcatLib