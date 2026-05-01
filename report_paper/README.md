# CIVIL 8025 Project 4 Report

This folder contains the LaTeX source for the research paper:

`A ROS 2 Digital Twin for Construction Robot Inspection`

## Build

Install Docker Desktop, then run from PowerShell:

```powershell
cd D:\HKU\8025_pj4\report_paper
.\build.ps1
```

The script builds the LaTeX Docker image and writes:

`output\CIVIL8025_Project4_Report_TianTan.pdf`

If Docker is not available, use the local MiKTeX build:

```powershell
cd D:\HKU\8025_pj4\report_paper
.\build_local.ps1
```

## Notes

- The report uses APA-style citations to match the CIVIL 8025 course overview.
- The paper describes the implemented project as a mobile inspection robot digital twin.
- It does not claim that a manipulator arm, MoveIt 2, real AprilTag detection, or physical deployment has been implemented.
