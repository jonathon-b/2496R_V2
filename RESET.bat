powershell -Command "(gc src/main.cpp) -replace 'blue_unprotected', 'edit_me_pls' | Out-File -encoding ASCII src/main.cpp"
powershell -Command "(gc src/main.cpp) -replace 'red_unprotected', 'edit_me_pls' | Out-File -encoding ASCII src/main.cpp"
powershell -Command "(gc src/main.cpp) -replace 'blue_protected', 'edit_me_pls' | Out-File -encoding ASCII src/main.cpp"
powershell -Command "(gc src/main.cpp) -replace 'red_protected', 'edit_me_pls' | Out-File -encoding ASCII src/main.cpp"
powershell -Command "(gc src/main.cpp) -replace 'one_cube', 'edit_me_pls' | Out-File -encoding ASCII src/main.cpp"
powershell -Command "(gc src/main.cpp) -replace 'skills', 'edit_me_pls' | Out-File -encoding ASCII src/main.cpp"