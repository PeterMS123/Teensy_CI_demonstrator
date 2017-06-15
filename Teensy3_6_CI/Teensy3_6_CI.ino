
#include "arm_math.h"
#include <ILI9341_t3.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>  


#define FS 22050 // sampling frequency
#define n_filters 22
#define FFT_SIZE 256   // Window size in samples
#define mask 255
#define max_average_periods 25 // Define the maximum number of main loop periods to average over when controlling electrode pulse rate

// Define analog input pins
#define mic_pin 33
#define n_electrode_pin 22
#define gain_pin 35
#define pulse_rate_pin 23

// Pinout for the SPI communication for the two touch displays
#define sclk 13  // Shared clock signal
#define miso 12  // Shared MISO
#define mosi 11  // shared MOSI

// Display 1, Spectrogram
#define cs1 15 
#define rst1 1
#define dc1 9  

// Display 2, electrodes
#define dc2 10 
#define cs2 20 
#define rst2 2

// Touch screen objects
ILI9341_t3 tft = ILI9341_t3(cs1, dc1, rst1, mosi, sclk, miso);
ILI9341_t3 tft2 = ILI9341_t3(cs2, dc2, rst2, mosi, sclk, miso);

#define NUMPIXELS 22 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    0
#define CLOCKPIN   32
Adafruit_DotStar strip = Adafruit_DotStar(
  NUMPIXELS, DATAPIN, CLOCKPIN);

float32_t magnitudes[FFT_SIZE/2]; // FFT output vector 

// 256 point hann-window
float32_t hann[FFT_SIZE] = {0.000000 ,0.000152 ,0.000607 ,0.001365 ,0.002427 ,0.003790 ,0.005454 ,0.007419 ,0.009683 ,0.012244 ,0.015102 ,0.018253 ,0.021698 ,0.025433 ,
0.029455 ,0.033764 ,0.038355 ,0.043227 ,0.048376 ,0.053800 ,0.059494 ,0.065456 ,0.071681 ,0.078166 ,0.084908 ,0.091902 ,0.099143 ,0.106628 ,0.114351 ,0.122309 ,0.130496 ,
0.138907 ,0.147537 ,0.156382 ,0.165435 ,0.174691 ,0.184144 ,0.193790 ,0.203621 ,0.213632 ,0.223818 ,0.234170 ,0.244684 ,0.255354 ,0.266171 ,0.277131 ,0.288226 ,0.299449 ,
0.310794 ,0.322255 ,0.333823 ,0.345492 ,0.357254 ,0.369104 ,0.381032 ,0.393033 ,0.405099 ,0.417223 ,0.429397 ,0.441614 ,0.453866 ,0.466146 ,0.478447 ,0.490761 ,0.503080 ,
0.515398 ,0.527706 ,0.539997 ,0.552264 ,0.564500 ,0.576696 ,0.588845 ,0.600941 ,0.612976 ,0.624941 ,0.636831 ,0.648638 ,0.660355 ,0.671974 ,0.683489 ,0.694893 ,0.706178 ,
0.717338 ,0.728366 ,0.739256 ,0.750000 ,0.760592 ,0.771027 ,0.781296 ,0.791395 ,0.801317 ,0.811056 ,0.820607 ,0.829962 ,0.839118 ,0.848067 ,0.856805 ,0.865327 ,0.873626 ,
0.881699 ,0.889540 ,0.897145 ,0.904508 ,0.911626 ,0.918495 ,0.925109 ,0.931464 ,0.937558 ,0.943387 ,0.948946 ,0.954233 ,0.959243 ,0.963976 ,0.968426 ,0.972592 ,0.976471 ,
0.980061 ,0.983359 ,0.986364 ,0.989074 ,0.991487 ,0.993601 ,0.995416 ,0.996930 ,0.998142 ,0.999052 ,0.999659 ,0.999962 ,0.999962 ,0.999659 ,0.999052 ,0.998142 ,0.996930 ,
0.995416 ,0.993601 ,0.991487 ,0.989074 ,0.986364 ,0.983359 ,0.980061 ,0.976471 ,0.972592 ,0.968426 ,0.963976 ,0.959243 ,0.954233 ,0.948946 ,0.943387 ,0.937558 ,0.931464 ,
0.925109 ,0.918495 ,0.911626 ,0.904508 ,0.897145 ,0.889540 ,0.881699 ,0.873626 ,0.865327 ,0.856805 ,0.848067 ,0.839118 ,0.829962 ,0.820607 ,0.811056 ,0.801317 ,0.791395 ,
0.781296 ,0.771027 ,0.760592 ,0.750000 ,0.739256 ,0.728366 ,0.717338 ,0.706178 ,0.694893 ,0.683489 ,0.671974 ,0.660355 ,0.648638 ,0.636831 ,0.624941 ,0.612976 ,0.600941 ,
0.588845 ,0.576696 ,0.564500 ,0.552264 ,0.539997 ,0.527706 ,0.515398 ,0.503080 ,0.490761 ,0.478447 ,0.466146 ,0.453866 ,0.441614 ,0.429397 ,0.417223 ,0.405099 ,0.393033 ,
0.381032 ,0.369104 ,0.357254 ,0.345492 ,0.333823 ,0.322255 ,0.310794 ,0.299449 ,0.288226 ,0.277131 ,0.266171 ,0.255354 ,0.244684 ,0.234170 ,0.223818 ,0.213632 ,0.203621 ,
0.193790 ,0.184144 ,0.174691 ,0.165435 ,0.156382 ,0.147537 ,0.138907 ,0.130496 ,0.122309 ,0.114351 ,0.106628 ,0.099143 ,0.091902 ,0.084908 ,0.078166 ,0.071681 ,0.065456 ,
0.059494 ,0.053800 ,0.048376 ,0.043227 ,0.038355 ,0.033764 ,0.029455 ,0.025433 ,0.021698 ,0.018253 ,0.015102 ,0.012244 ,0.009683 ,0.007419 ,0.005454 ,0.003790 ,0.002427 ,
0.001365 ,0.000607 ,0.000152 ,0.000000 };

// Filterbank coefficients [start index, stop index, real[0], imag[0], real[1], imag[1]....
float32_t FIR[22][126] = {{2 ,6 ,0.002102, -0.015641,0.000000, 0.000000,0.829182, -0.497080,-0.062376, 0.066439,0.002757, 0.010879},
{3 ,7 ,-0.003818, -0.029175,-0.259972, 0.007782,0.554986, -0.720197,-0.078306, 0.069724,0.002108, 0.015187},
{3 ,8 ,0.002933, -0.009083,-0.012683, -0.040068,-0.299146, 0.063774,0.741014, -0.590023,-0.141834, 0.064440,-0.001571, 0.024945},
{4 ,10 ,0.002686, -0.011933,-0.017147, -0.044186,-0.272237, 0.024893,0.998072, 0.062068,-0.287292, -0.045496,-0.019363, 0.046034,0.002552, 0.012482},
{5 ,12 ,0.002508, -0.013057,-0.014137, -0.041436,-0.192654, -0.043677,0.356094, 0.770853,-0.229943, -0.561235,-0.101581, 0.070130,-0.004359, 0.029704,0.002818, 0.009477},
{6 ,14 ,0.002608, -0.012318,-0.007453, -0.034041,-0.100790, -0.070093,-0.324666, 0.389248,0.922320, -0.343515,-0.331699, -0.142909,-0.058795, 0.065374,-0.002425, 0.026475,0.002806, 0.009716},
{7 ,16 ,0.002803, -0.010407,-0.001711, -0.025183,-0.039447, -0.058393,-0.237016, -0.012187,0.066177, 0.751902,0.498234, -0.734044,-0.304841, -0.075183,-0.062037, 0.066209,-0.005048, 0.030726,0.002531, 0.012588},
{9 ,19 ,0.001502, -0.017334,-0.010895, -0.038121,-0.078638, -0.069197,-0.304976, 0.074991,0.248792, 0.778940,0.546936, -0.715005,-0.338017, -0.166912,-0.103319, 0.070106,-0.016042, 0.043211,0.000678, 0.019760,0.002824, 0.009205},
{10 ,21 ,0.002702, -0.011462,-0.000498, -0.022686,-0.018025, -0.044941,-0.092549, -0.070174,-0.299093, 0.064308,0.016945, 0.736038,0.931542, -0.326026,-0.280427, -0.489533,-0.216515, 0.028675,-0.055302, 0.064431,-0.008870, 0.035796,0.001309, 0.017893},
{12 ,25 ,0.002444, -0.013258,-0.001256, -0.024267,-0.017220, -0.044222,-0.074741, -0.068626,-0.234802, -0.014089,-0.299851, 0.451817,0.748792, 0.579737,0.441954, -0.752074,-0.344613, -0.292667,-0.187126, 0.046598,-0.054850, 0.064246,-0.011469, 0.038701,0.000159, 0.021081,0.002657, 0.011592},
{14 ,29 ,0.002473, -0.013083,-0.000299, -0.022217,-0.010712, -0.037902,-0.044051, -0.060424,-0.137458, -0.065309,-0.311321, 0.087999,-0.164376, 0.628085,0.880965, 0.420200,0.435708, -0.754246,-0.330377, -0.370911,-0.236584, 0.012675,-0.088391, 0.069991,-0.025816, 0.050775,-0.004892, 0.030499,0.001322, 0.017836,0.002752, 0.010607},
{16 ,33 ,0.002697, -0.011402,0.001269, -0.018042,-0.003861, -0.028915,-0.018961, -0.045686,-0.059543, -0.065569,-0.155936, -0.059959,-0.311898, 0.089506,-0.233117, 0.556965,0.644374, 0.660911,0.837680, -0.482032,-0.114099, -0.666603,-0.336435, -0.161198,-0.186378, 0.046993,-0.074137, 0.068547,-0.024676, 0.049982,-0.005917, 0.031964,0.000600, 0.019943,0.002532, 0.012559},
{19 ,38 ,0.002401, -0.013475,0.000491, -0.020264,-0.005016, -0.030682,-0.019072, -0.045774,-0.052621, -0.063540,-0.126051, -0.067580,-0.255374, 0.005546,-0.344342, 0.295293,0.027828, 0.739309,0.883460, 0.415110,0.702199, -0.619514,-0.135561, -0.650998,-0.342839, -0.196371,-0.219410, 0.026378,-0.102811, 0.070043,-0.041575, 0.059349,-0.014358, 0.041614,-0.003135, 0.027703,0.001168, 0.018307,0.002579, 0.012217},
{21 ,44 ,0.002828, -0.009555,0.002396, -0.013504,0.000841, -0.019314,-0.003184, -0.027806,-0.012521, -0.039809,-0.032995, -0.055125,-0.075613, -0.068762,-0.156487, -0.059791,-0.277795, 0.032135,-0.342739, 0.309720,-0.036362, 0.712282,0.733347, 0.593917,0.928887, -0.329985,0.197605, -0.776033,-0.301514, -0.448537,-0.316606, -0.099939,-0.194036, 0.042881,-0.097674, 0.070175,-0.044092, 0.060429,-0.017739, 0.044669,-0.005527, 0.031417,-0.000148, 0.021825,0.002030, 0.015216,0.002741, 0.010722},
{25 ,50 ,0.002636, -0.011906,0.001791, -0.016238,-0.000349, -0.022316,-0.005052, -0.030740,-0.014749, -0.041996,-0.033964, -0.055653,-0.070536, -0.067972,-0.135600, -0.065711,-0.235379, -0.013650,-0.335157, 0.155612,-0.281118, 0.487608,0.173370, 0.773659,0.852437, 0.462422,0.901388, -0.384814,0.245373, -0.778258,-0.255581, -0.527497,-0.340893, -0.183110,-0.247201, 0.002818,-0.144492, 0.063498,-0.075819, 0.068798,-0.036819, 0.057133,-0.016222, 0.043345,-0.005789, 0.031789,-0.000706, 0.023085,0.001627, 0.016791,0.002569, 0.012302},
{29 ,58 ,0.002558, -0.012479,0.001745, -0.016403,-0.000081, -0.021685,-0.003754, -0.028738,-0.010754, -0.037931,-0.023660, -0.049268,-0.046791, -0.061487,-0.086731, -0.069854,-0.151291, -0.061466,-0.242175, -0.007511,-0.331036, 0.140580,-0.314338, 0.418409,-0.013427, 0.722805,0.586119, 0.694842,0.995020, 0.089234,0.720662, -0.604515,0.104166, -0.761591,-0.277521, -0.493473,-0.342221, -0.191984,-0.264506, -0.015636,-0.169737, 0.054746,-0.098875, 0.070146,-0.054047, 0.063997,-0.027788, 0.052039,-0.013034, 0.040318,-0.004981, 0.030617,-0.000720, 0.023111,0.001430, 0.017467,0.002420, 0.013270,0.002780, 0.010156},
{33 ,66 ,0.002692, -0.011418,0.002205, -0.014482,0.001130, -0.018465,-0.000949, -0.023627,-0.004725, -0.030253,-0.011339, -0.038571,-0.022645, -0.048543,-0.041559, -0.059349,-0.072365, -0.068276,-0.120446, -0.068474,-0.189847, -0.045175,-0.275025, 0.028494,-0.341741, 0.188466,-0.301742, 0.448009,-0.028970, 0.715825,0.488423, 0.737609,0.942006, 0.299360,0.908322, -0.371821,0.422310, -0.757395,-0.075565, -0.691213,-0.316112, -0.414044,-0.337193, -0.164373,-0.264982, -0.016134,-0.180671, 0.049857,-0.113781, 0.069288,-0.067994, 0.067503,-0.038842, 0.058112,-0.021010, 0.047314,-0.010379, 0.037516,-0.004176, 0.029401,-0.000647, 0.022960,0.001285, 0.017951,0.002273, 0.014088,0.002708, 0.011117},
{38 ,76 ,0.002675, -0.011581,0.002249, -0.014270,0.001387, -0.017657,-0.000176, -0.021914,-0.002851, -0.027235,-0.007279, -0.033802,-0.014450, -0.041708,-0.025866, -0.050791,-0.043736, -0.060272,-0.071109, -0.068057,-0.111685, -0.069469,-0.168573, -0.055221,-0.240519, -0.009030,-0.313350, 0.092700,-0.346080, 0.271864,-0.261900, 0.518090,0.021589, 0.737035,0.485895, 0.738322,0.905237, 0.377226,0.970850, -0.213977,0.627834, -0.671219,0.144233, -0.769378,-0.202139, -0.591890,-0.337759, -0.338923,-0.329802, -0.136425,-0.262863, -0.013752,-0.188298, 0.045978,-0.126527, 0.067505,-0.081422, 0.069436,-0.050590, 0.062868,-0.030301, 0.053581,-0.017266, 0.044260,-0.009040, 0.035975,-0.003933, 0.029020,-0.000827, 0.023353,0.001009, 0.018806,0.002042, 0.015184,0.002574, 0.012308,0.002795, 0.010022},
{43 ,87 ,0.002790, -0.010353,0.002574, -0.012407,0.002115, -0.014921,0.001285, -0.017999,-0.000106, -0.021762,-0.002340, -0.026340,-0.005831, -0.031862,-0.011188, -0.038412,-0.019296, -0.045965,-0.031412, -0.054232,-0.049268, -0.062406,-0.075102, -0.068682,-0.111471, -0.069489,-0.160479, -0.058356,-0.221756, -0.024616,-0.288221, 0.047076,-0.339070, 0.173691,-0.332425, 0.362266,-0.207987, 0.585719,0.080725, 0.755828,0.497718, 0.734096,0.875792, 0.427434,0.995756, -0.082581,0.777865, -0.551325,0.363558, -0.769226,-0.025738, -0.717205,-0.262430, -0.517292,-0.344018, -0.298370,-0.327162, -0.128129,-0.268141, -0.019952,-0.201855, 0.038225,-0.144018, 0.063592,-0.099027, 0.070140,-0.066163, 0.067124,-0.043045, 0.059986,-0.027168, 0.051650,-0.016444, 0.043547,-0.009295, 0.036288,-0.004591, 0.030058,-0.001540, 0.024838,0.000398, 0.020524,0.001592, 0.016985,0.002291, 0.014093,0.002664, 0.011730,0.002823, 0.009799},
{49 ,99 ,0.002829, -0.009859,0.002694, -0.011545,0.002401, -0.013558,0.001879, -0.015962,0.001025, -0.018834,-0.000305, -0.022258,-0.002315, -0.026323,-0.005291, -0.031117,-0.009634, -0.036701,-0.015902, -0.043079,-0.024861, -0.050130,-0.037537, -0.057494,-0.055263, -0.064381,-0.079658, -0.069261,-0.112473, -0.069403,-0.155101, -0.060238,-0.207470, -0.034666,-0.265902, 0.017291,-0.319623, 0.107449,-0.346485, 0.245128,-0.310799, 0.427173,-0.170165, 0.622580,0.100141, 0.760682,0.467429, 0.744502,0.817572, 0.507234,0.995515, 0.085402,0.909153, -0.369998,0.604201, -0.685103,0.226071, -0.777611,-0.087294, -0.684056,-0.274076, -0.499148,-0.343017, -0.306730,-0.334014, -0.151542,-0.286736, -0.044899,-0.228257, 0.019501,-0.172984, 0.053323,-0.126692, 0.067452,-0.090442, 0.070030,-0.063197, 0.066466,-0.043256, 0.060077,-0.028921, 0.052763,-0.018748, 0.045538,-0.011605, 0.038892,-0.006637, 0.033015,-0.003217, 0.027940,-0.000893, 0.023619,0.000659, 0.019972,0.001670, 0.016909,0.002302, 0.014343,0.002672, 0.012195,0.002861, 0.010396},
{56 ,113 ,0.002869, -0.009678,0.002772, -0.011111,0.002565, -0.012785,0.002204, -0.014742,0.001630, -0.017029,0.000761, -0.019702,-0.000510, -0.022819,-0.002332, -0.026442,-0.004904, -0.030628,-0.008494, -0.035421,-0.013461, -0.040835,-0.020278, -0.046818,-0.029565, -0.053206,-0.042116, -0.059641,-0.058906, -0.065444,-0.081075, -0.069432,-0.109814, -0.069646,-0.146092, -0.063018,-0.190092, -0.044990,-0.240183, -0.009271,-0.291269, 0.051923,-0.332582, 0.146453,-0.345571, 0.278673,-0.303820, 0.442915,-0.178419, 0.614895,0.047750, 0.745975,0.358372, 0.769931,0.687625, 0.630675,0.931547, 0.323567,0.996014, -0.080521,0.854056, -0.459839,0.563949, -0.706068,0.230980, -0.777753,-0.051833, -0.704178,-0.238506, -0.549970,-0.328346, -0.376592,-0.345351, -0.223104,-0.318619, -0.105561,-0.271764, -0.024785,-0.220137, 0.025545,-0.172025, 0.053540,-0.130940, 0.066491,-0.097653, 0.070007,-0.071583, 0.068026,-0.051626, 0.063142,-0.036591, 0.056975,-0.025394, 0.050488,-0.017131, 0.044215,-0.011077, 0.038427,-0.006673, 0.033238,-0.003494, 0.028667,-0.001217, 0.024687,0.000395, 0.021248,0.001520, 0.018289,0.002291, 0.015748,0.002803, 0.013568,0.003128, 0.011697,0.003321, 0.010088},
{64 ,125 ,0.002984, -0.009603,0.002917, -0.010834,0.002771, -0.012245,0.002519, -0.013862,0.002127, -0.015716,0.001546, -0.017843,0.000718, -0.020281,-0.000438, -0.023069,-0.002024, -0.026251,-0.004175, -0.029864,-0.007066, -0.033942,-0.010921, -0.038501,-0.016029, -0.043525,-0.022760, -0.048948,-0.031575, -0.054619,-0.043044, -0.060249,-0.057847, -0.065345,-0.076759, -0.069099,-0.100593, -0.070257,-0.130074, -0.066935,-0.165587, -0.056428,-0.206736, -0.035031,-0.251647, 0.001999,-0.295962, 0.060136,-0.331621, 0.144602,-0.345788, 0.258271,-0.320810, 0.398173,-0.236640, 0.550914,-0.077299, 0.688822,0.158417, 0.770492,0.445457, 0.750172,0.728455, 0.597081,0.933956, 0.317319,0.999198, -0.036848,0.902791, -0.382292,0.676128, -0.639632,0.386796, -0.765402,0.106536, -0.762214,-0.114946, -0.665677,-0.258524, -0.521939,-0.329317, -0.370091,-0.344848, -0.234804,-0.325058, -0.127028,-0.286733, -0.048231,-0.241703, 0.005161,-0.197196, 0.038544,-0.156982, 0.057296,-0.122570, 0.065960,-0.094154, 0.068031,-0.071254, 0.066034,-0.053114, 0.061706,-0.038919, 0.056200,-0.027912, 0.050244,-0.019434, 0.044282,-0.012939, 0.038563,-0.007987, 0.033213,-0.004227, 0.028279,-0.001387, 0.023759,0.000744, 0.019624,0.002325, 0.015829,0.003479, 0.012319,0.004294, 0.009039}};

//Relative filterbank output scaling to give equal response to pure tone at center frequency
//float32_t filter_scale[22] = {1, 1, 1,1, 1,1, 1, 1,1, 1,1, 1, 1,1, 1,1, 1, 1,1, 1,1,1 }; // TEST
//float32_t filter_scale[22] = {0.7, 0.7, 0.857 ,0.8 ,0.857 ,0.8 ,0.909 ,0.967 , 0.8 ,1.111 ,1 , 1.3, 2.2, 2, 1.2, 0.9, 1, 1, 1.2 ,1.5 ,2.8 ,3}; // Left
float32_t filter_scale[22] = {0.9, 0.9, 1 ,1 ,1 ,0.9 ,1.4 ,1.6 , 1.3 ,1.8 ,3 , 6, 5, 2.5, 2.5, 2, 2, 2.2, 2.3 ,3 ,8 ,10}; // Right

// Real variables
int x[FFT_SIZE]; // Circular input buffer
int DC_offset = 500; // inital DC offset of input signal
int j = 0; // Current index in input buffer
float32_t mag_filt[n_filters]; // The heigth of each frequency bar for the display in pixels
int mag_filt_index[n_filters]; // Index array of electrodes after filtering

// Circular buffer for storing the most recently spectrogram values. used to calclulate the output when the pulserate is reduced
float32_t mag_electrode_buffer[n_filters][max_average_periods]; 
float32_t mag_electrode[n_filters] = {0};
int j_electrode = 0; // Current index in the electrode circular buffer.

// Spectrogram colours
uint16_t colors[33] = {ILI9341_BLUE, 0x011F, 0x021F, 0x031F, 0x041F, 0x04FF, 0x05FF,0x06FF, ILI9341_CYAN,0x07FB, 0x07F7,0x07F3, 0x07F0, 0x07EC, 0x07E8, 0x07E4, ILI9341_GREEN, 0x27E0, 0x47E0, 0x67E0, 0x87E0, 0x9FE0, 0xBFE0, 0xDFE0, ILI9341_YELLOW, 0xFEE0, 0xFDE0, 0xFCE0, 0xFC00, 0xFB00, 0xFA00, 0xF900, ILI9341_RED};

int M_of_N = 2; // Number of active electrodes
// maximum magnitude of filtered signals. Adjust this to adjust overall gain
float32_t max_amp = 60; // 18 for left Teensy, 60 for right
//Filter dependent noise floor cutoff
//float32_t noise_cutoff[n_filters] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // TEST 
//float32_t noise_cutoff[n_filters] = {0.4 , 0.35, 0.35 ,0.21 ,0.20 ,0.12 ,0.12 ,0.15 , 0.12  ,0.12  ,0.12, 0.15 , 0.2 ,0.08, 0.05 , 0.05 , 0.03 , 0.03 , 0.03 ,0.05 ,0.10 ,0.10}; // Left 
float32_t noise_cutoff[n_filters] = {0.10,0.10 ,0.05, 0.05 ,0.05 , 0.05 , 0.05 , 0.05 , 0.05 ,0.05 ,0.05 ,0.05 ,0.05 ,0.05,0.05 ,0.05,0.05,0.03 ,0.03, 0.05 ,0.10,0.10}; // right
int pulse_rate_periods = 1;

float32_t envelope[FFT_SIZE]; // Envelope

// Execution time measurement variables
long int start_t;  
long int stop_t;

arm_cfft_radix4_instance_f32 S;    /* ARM CFFT module */
arm_cfft_radix4_instance_f32 S_ifft;    /* ARM CFFT module */

IntervalTimer sampleTimer; // Create sampling timer object

int count_mainLoop = 0;
int count_pulseRate = 0;
float32_t exeTime; // Main loop execution time
int gain_prev;
float32_t sec_size; // Section size for electrodogram 

// Color values for each LED
int LED_color[22] = { 0x000101,0x000101,0x000101, 0x000002,0x000002,0x000002, 0x010001,0x010001,0x010001,0x010001, 0x020000,0x020000,0x020000,0x020000, 0x010100,0x010100,0x010100,0x010100, 0x000200, 0x000200, 0x000200, 0x000200}; // Specify
int LED_intensity[10] = {0,1,2, 3, 6, 12, 25, 50, 80, 127}; // Discretized intensities for the LEDs


// Sampling interrupt handler, triggered by sampleTimer object
void sampleInput(){
 x[j] = analogRead(mic_pin)- DC_offset; 
 j = (j+1) & mask ; // Update circular buffer index
}

// Setup function. This is run once when the Teensy is restarted
void setup() {

  strip.begin(); // Initialize pins for output
  delay(200);
  
 // Serial.begin(9600);
  float T_micros = (float)1000000/FS; // Sampling period in micro seconds
  sampleTimer.begin(sampleInput, T_micros); // Init timer to generate interrupt at 22.05 kHz
  tft.begin(); // Init display 1
  tft2.begin(); // Init display 2
 
  meas_offset(); // measure the DC voltage offset of the microphone input
  //Serial.println(DC_offset);

  arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1); // Init FFT instance

  arm_cfft_radix4_init_f32(&S_ifft, FFT_SIZE, 1, 1); // Init IFFT instance

  init_displays(); // Write the initial text and background colors for the displays
}


//**************************** MAIN LOOP BEGINS *******************************************
void loop() {
  
  // Read pot for M of N active electrodes
  int m_temp = analogRead(n_electrode_pin)/45;
  if(M_of_N != m_temp){
    M_of_N = m_temp;
    tft2.fillRect(221,0, 19,25,ILI9341_BLACK);
    tft2.setRotation(1);
    tft2.setCursor(0, 2);
    tft2.print(M_of_N);
    tft2.setRotation(0);
  }

  /*
  // Read pot for overall gain
  int gain_temp = analogRead(gain_pin);
  if(gain_temp != gain_prev){
    max_amp = 55 -(float32_t)gain_temp/20.0;
    gain_prev = gain_temp;
    //Serial.println(max_amp);
  }
  
*/
  // Read pot for electrode pulse rate
  int pulse_rate_temp = (analogRead(pulse_rate_pin)*max_average_periods)/1024 +1;

  if(pulse_rate_periods != pulse_rate_temp){
    pulse_rate_periods = pulse_rate_temp;

    float32_t PPS = 1000/(exeTime*pulse_rate_periods);
    if(PPS > 500){
      PPS = 0;
    }
    tft2.fillRect(221,200, 19,95,ILI9341_BLACK);
    tft2.setRotation(1);
    tft2.setCursor(200, 2);
    tft2.print(PPS);
    tft2.setRotation(0);
  }



  // Scroll displays by one pixel
  tft.setScroll(count_mainLoop%320 + 1);
  tft2.setScroll(count_mainLoop%320 + 1);

  // Calculate main loop execution time
  if(count_mainLoop%100 == 0){
    stop_t = millis();
    int delta_t = stop_t - start_t;
    exeTime = delta_t/100.0;
    //Serial.print(exeTime);
    //Serial.println(" ms execution time");
    start_t = millis();     
  }
  count_mainLoop++; // Update main loop count 

 float32_t windowed_data[FFT_SIZE];
 float32_t input_c[FFT_SIZE*2];
 applyWindow(0, FFT_SIZE, windowed_data); // Apply Hann window to the time frame copied from the input buffer

 // Add an imaginary part of 0
 for (int i = 0; i < FFT_SIZE; i++){
  input_c[2*i] = windowed_data[i]*4;
  input_c[2*i +1] = 0;
 }
 
 // Do the single FFT of the signal - Then it gets træls
 arm_cfft_radix4_f32(&S, input_c);  // FFT result is now stored in input_c array

 // apply filter, do IFFT, calculate mean of envelope
 for (int i = 0; i < n_filters; i++){

  int start_i = FIR[i][0];
  int stop_i = FIR[i][1];
  int n_points = stop_i - start_i +1;

  float32_t filt_sig[FFT_SIZE*2] = {0};
  
  // Hopefully this only multiplies the right fft values with the right filter values and stores it in the right position of IFFT vector. Hopefully
  arm_cmplx_mult_cmplx_f32(input_c + start_i*2, FIR[i]+2, filt_sig + start_i*2, n_points); 

  // IFFT of the filtered signal with zero negative frequency components! Result stored in filt_sig is the analytical signal
  arm_cfft_radix4_f32(&S_ifft, filt_sig);
  
  arm_cmplx_mag_f32(filt_sig, envelope, FFT_SIZE); // Envelope is magnitude of analytical signal
  arm_mean_f32(envelope, FFT_SIZE, mag_filt+i); // Mean of envelope is considered instantanous envelope value
  mag_filt[i] = mag_filt[i]*filter_scale[i]; // Scale outputs slightly to give even filter outputs at center frequencies

  // Ensure magnitude is within max amplitude
  mag_filt[i] = fit_val(mag_filt[i], max_amp*noise_cutoff[i], max_amp);
  
  mag_electrode_buffer[i][j_electrode] = mag_filt[i]; // Copy filter output to electrode buffer 
 }
  
  drawSG(count_mainLoop%320); // Draw spectrogram with all filter ouputs present
 
  // If the pulse rate is reduced, only update the electrode values every N times. Output is the average of last N periods
  if(count_mainLoop%pulse_rate_periods == 0){
    //Serial.println(pulse_rate_periods);
    // Calculate the mean of last N outputs for each filter
    for(int i = 0; i <n_filters; i++){
      mag_filt_index[i] = i; // Reset the index array
      mag_electrode[i] = 0;
      for(int m = 0; m < pulse_rate_periods; m++){
        mag_electrode[i] += mag_electrode_buffer[i][(j_electrode - m + max_average_periods)%max_average_periods];
      }
      mag_electrode[i] = mag_electrode[i]/pulse_rate_periods;
    }


  // Sort the filter output magnitudes and set all but the M_of_N highest values to zero
    sort(mag_electrode, mag_filt_index);
    for(int i = 0; i < n_filters - M_of_N; i++){
      mag_electrode[i] = 0;
    } 

    //Discretize magnitudes, to integer values between 0 and 9 to fit with the electrodogram
    // Write LED strip values
    for (int i = 0; i<n_filters; i++){
      mag_electrode[i] = round(mag_electrode[i]*9/max_amp);
      strip.setPixelColor(n_filters - mag_filt_index[i],set_brightness(LED_color[mag_filt_index[i]], LED_intensity[(int)mag_electrode[i]]));  
    }
    strip.show(); 
    
  }

  drawElectrode(count_mainLoop%320); // Draw electrodogram

  j_electrode = (j_electrode+1)%max_average_periods; // update index in the electrude buffer array


  
}

// ********************** END OF MAIN LOOP *********************************************


//*********************** FUNCTION DECLARATIONS ***********************************************

void applyWindow(int start_index, int stop_index, float32_t *windowed_data){
  int offset_ = j;
  for(int i = start_index; i < stop_index; i++){
    windowed_data[i] = (x[(FFT_SIZE + offset_ +(i-mask))%FFT_SIZE]*hann[i]); // Finds the right sample in the input buffer and multiplies it by the corresponding value in the hann-window
  }
}


void meas_offset(){
  int s = 0;
  for (int i = 0; i <1000; i++){
    analogReadResolution(10);
    s = s + analogRead(mic_pin);
  }
  DC_offset = s/1000;
}


void drawSG(int pos){
 
  int n_colors = 33;
  float32_t section_size = max_amp/n_colors;
  int color_intensity;
  for(int i = 0; i < n_filters; i++){
    color_intensity = floor(mag_filt[i]/section_size);
    tft.fillRect(i*10, pos, 10, 1, colors[color_intensity]); 
  }
}
/*
 * Draws one vertical line of the electrodogram, based on the bar heights in pixels stored in the mag_electrode array
 */
void drawElectrode(int pos){
  for(int i = 0; i < n_filters; i++){
    int vert_pos = mag_filt_index[i]*10;
    tft2.drawPixel(vert_pos, pos, ILI9341_NAVY);
    if (mag_electrode[i] > 0){
      tft2.fillRect(vert_pos + 1, pos, mag_electrode[i], 1, ILI9341_WHITE);
    }
    if(mag_electrode[i] != 9){
       tft2.fillRect(vert_pos + 1 + mag_electrode[i], pos, 9 - mag_electrode[i], 1, ILI9341_BLACK);
    }
  }
}



/*
  Cuts off the values of x so that min_val <= x <= max_val + min_val, and moves x to the range 0 <= x <= max_val
  This function ensures the led-outputs are in an allowed range.
INPUTS:
  int x - The value to be fitted
  int min_val - The minimal cutoff value
  int max_val - The maximum value of the new range
OUTPUTS:
  int x - The now fitted value
*/
float32_t fit_val(float32_t x, float32_t min_val, float32_t max_val){
  x = x - min_val;
  if(x < 0){
    x = 0;
  }else if(x > max_val){
    x = max_val*0.99;
  }
  return x;
}

/*
  Sorts the elements in the x-array from lowest to highest, and keeps track of their indexes in the index array.
INPUTS:
  flaot32_t x[22] - An array of 22 float to be sorted.
  int index[22] - The indexes of the elements in the x-array
OUTPUTS:
  nothing...
*/
void sort(float32_t x[22], int index[22]){
  float32_t temp;
  int j;
  for(int i =1; i < 22; i++){
    temp = x[i];
    j = i -1;
    while(j >= 0 && x[j] > temp){ // Looking for the right place to insert the element
      x[j+1] = x[j];
      index[j+1] = index[j];
      j--;     
    }
    x[j+1] = temp;
    index[j+1] = i;   
  }  
}



void init_displays(){
  /*
  for(int i = 0; i < 33; i++){
    tft.fillRect(10, 20, 10, 10, colors[i]); 
    delay(200);
  }
  */
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLUE);
  tft.setCursor(50, 2);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
  tft.print("Filterbank output");
  tft.setRotation(0);
  tft.drawLine(220,0, 220,320, ILI9341_RED);
  

  tft2.setRotation(1);
  tft2.fillScreen(ILI9341_BLACK);
  tft2.setCursor(0, 2);
  tft2.setTextColor(ILI9341_WHITE);  tft2.setTextSize(2);
  
  tft2.print(M_of_N);
  tft2.print("  of 22 - PPS =" );
  delay(500);
  tft2.setRotation(0);
}

/*
 *  Calculates the corresponding BRG value, given a base color and an integer intenisty
 *  INPUT: 
 *    - int color : Base color value. Blue is fx 0f010000
 *    - int intensity : The amount to multiply each component of the base color with
 *  RETURN:
 *    - int : the color value after each component has been multiplied by the intensity 
 */
int set_brightness(int color, int intensity){
  int blue = color & 0xFF0000;
  int red = color & 0x00FF00;
  int green = color & 0x0000FF;
  //Serial.println(blue);
  //Serial.println(red);
  //Serial.println(green);
  //Serial.println("BRG");
  
  return blue*intensity + red*intensity + green *intensity;

}
