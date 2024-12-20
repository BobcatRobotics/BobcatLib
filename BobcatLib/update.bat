@echo off
echo Formatting
call gradlew.bat :spotlessApply
echo Publish
call gradlew.bat publish