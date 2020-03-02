powershell -Command "(gc src/main.cpp) -replace 'edit_me_pls', 'blue_unprotected' | Out-File -encoding ASCII src/main.cpp"
prosv5 make
prosv5 upload --slot 1 --name "BLUE_BIG"
powershell -Command "(gc src/main.cpp) -replace 'blue_unprotected', 'red_unprotected' | Out-File -encoding ASCII src/main.cpp"
prosv5 make
prosv5 upload --slot 2 --name "RED_BIG"
powershell -Command "(gc src/main.cpp) -replace 'red_unprotected', 'blue_protected' | Out-File -encoding ASCII src/main.cpp"
prosv5 make
prosv5 upload --slot 3 --name "RED_SMALL"
powershell -Command "(gc src/main.cpp) -replace 'blue_protected', 'red_protected' | Out-File -encoding ASCII src/main.cpp"
prosv5 make
prosv5 upload --slot 4 --name "BLUE_SMALL"
powershell -Command "(gc src/main.cpp) -replace 'red_protected', 'skills' | Out-File -encoding ASCII src/main.cpp"
prosv5 make
prosv5 upload --slot 5 --name "SKILLS"
powershell -Command "(gc src/main.cpp) -replace 'skills', 'one_cube' | Out-File -encoding ASCII src/main.cpp"
prosv5 make
prosv5 upload --slot 6 --name "ONE_CUBE"
powershell -Command "(gc src/main.cpp) -replace 'one_cube', 'blue_unprotected' | Out-File -encoding ASCII src/main.cpp"