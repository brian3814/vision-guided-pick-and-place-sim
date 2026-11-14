# Stop the ROS 2 side of the cell started with locator.launch.py: the launch process, the
# object_locator node (its .exe launcher and the python.exe it starts), the static transform
# publisher and RViz2. Only processes of those programs are matched.
#   powershell -ExecutionPolicy Bypass -File ros2_ws\stop_locator.ps1
$names = @('python.exe', 'object_locator.exe', 'static_transform_publisher.exe', 'ros2.exe', 'rviz2.exe')
$procs = Get-CimInstance Win32_Process | Where-Object {
    $names -contains $_.Name -and $_.CommandLine -match 'pick_place_ros|static_transform_publisher|rviz2'
}
foreach ($p in $procs) {
    Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
    "stopped {0} ({1})" -f $p.Name, $p.ProcessId
}
if (-not $procs) { "nothing to stop" }
