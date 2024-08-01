function [Down_dir,Omega_dir,Other_dir] = MagID2Orbit(Inclination,Declination, Orbit_Incl)

Down_dir = sin(Inclination);
Omega_dir = cos(Inclination)*cos(Declination - Orbit_Incl);
Other_dir = cos(Inclination)*sin(Declination - Orbit_Incl);
end

