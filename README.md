# mav2simu_link
Routines for getting mavlink packets from raw data in Matlab and simulink real-time for use in simulations

Please use the included mavlink repo. Its not the bleeding edge latest, but quite recent(april 2015 clone). MSVC has issues with mavlink code as it enforces C89 style syntax. (mainly for "inline"->"__inline" and declaration of all vars at the start of functions.)

The mex_test and sfunction_test code has been tested with Matlab R2013. Mex_test works perfectly, sfunction_test crashes matlab at the end of simulation. Need to look into it.

The blocks do not parse received data yet, but it is on the roadmap.(It should quite easy to code it in too since readymade functions are available in mavlink repo)

These blocks were created specifially for simulation purposes and NOT for flight.

Code is shared on as is basis and totally at your risk.
