@echo off
echo Copy
Xcopy /E /y .\\BobcatLib\\build\\repos\\releases\\BobcatLib .\\vendordeps\BobcatLib\\repos\\BobcatLib
Xcopy /E /y .\\BobcatLib\\build\\docs\\javadoc .\\docs\\api-docs