function [Down_dir,Omega_dir,Other_dir] = MagID2NED(Inclination,Declination)

Down_dir = sin(Inclination);
Omega_dir = cos(Inclination)*cos(Declination);
Other_dir = cos(Inclination)*sin(Declination);
end

