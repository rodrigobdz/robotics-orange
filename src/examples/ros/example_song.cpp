#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/StoreSong.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_song");
	ros::NodeHandle n;
	ros::ServiceClient storeSong = n.serviceClient<create_fundamentals::StoreSong>("store_song");
	ros::ServiceClient playSong = n.serviceClient<create_fundamentals::PlaySong>("play_song");

	create_fundamentals::StoreSong srv;

	// define silence
	unsigned int r = 30;

	// map note names in the lilypad notation to irobot commands
	unsigned int c4 = 60;
	unsigned int cis4 = 61;
	unsigned int des4 = cis4;
	unsigned int d4 = 62;
	unsigned int dis4 = 63;
	unsigned int ees4 = dis4;
	unsigned int e4 = 64;
	unsigned int f4 = 65;
	unsigned int fis4 = 66;
	unsigned int ges4 = fis4;
	unsigned int g4 = 67;
	unsigned int gis4 = 68;
	unsigned int aes4 = gis4;
	unsigned int a4 = 69;
	unsigned int ais4 = 70;
	unsigned int bes4 = ais4;
	unsigned int b4 = 71;
	unsigned int c5 = 72;
	unsigned int cis5 = 73;
	unsigned int des5 = cis5;
	unsigned int d5 = 74;
	unsigned int dis5 = 75;
	unsigned int ees5 = dis5;
	unsigned int e5 = 76;
	unsigned int f5 = 77;
	unsigned int fis5 = 78;
	unsigned int ges5 = fis5;
	unsigned int g5 = 79;
	unsigned int gis5 = 80;
	unsigned int aes5 = gis5;
	unsigned int a5 = 81;
	unsigned int ais5 = 82;
	unsigned int bes5 = ais5;
	unsigned int b5 = 83;
	unsigned int c6 = 84;
	unsigned int cis6 = 85;
	unsigned int des6 = cis6;
	unsigned int d6 = 86;
	unsigned int dis6 = 87;
	unsigned int ees6 = dis6;
	unsigned int e6 = 88;
	unsigned int f6 = 89;
	unsigned int fis6 = 90;
	unsigned int ges6 = fis6;

	// define some note lengths
	// change the top MEASURE (4/4 time) to get faster/slower speeds
	unsigned int MEASURE = 160;
	unsigned int HALF = MEASURE/2;
	unsigned int Q = MEASURE/4;
	unsigned int E = MEASURE/8;
	unsigned int Ed = MEASURE*3/16;
	unsigned int S = MEASURE/16;

	float MEASURE_TIME = (float)(MEASURE)/64;

	create_fundamentals::StoreSong storeSongService;
	create_fundamentals::PlaySong playSongService;

	storeSongService.request.number = 0;
	storeSongService.request.song = {r,Q};
	storeSong.call(storeSongService);

	storeSongService.request.number = 1;
	storeSongService.request.song = {a4,Q, a4,Q, a4,Q, f4,Ed, c5,S, a4,Q, f4,Ed, c5,S, a4,HALF};
	storeSong.call(storeSongService);

	storeSongService.request.number = 2;
	storeSongService.request.song = {e5,Q, e5,Q, e5,Q, f5,Ed, c5,S,aes4,Q, f4,Ed, c5,S, a4,HALF};
	storeSong.call(storeSongService);

	storeSongService.request.number = 3;
	storeSongService.request.song = {a5,Q, a4,Ed, a4,S, a5,Q, aes5,E, g5,E,ges5,S, f5,S, ges5,S};
	storeSong.call(storeSongService);

	storeSongService.request.number = 4;
	storeSongService.request.song = {r,E, bes4,E, ees5,Q, d5,E, des5,E,c5,S, b4,S, c5,E};
	storeSong.call(storeSongService);

	storeSongService.request.number = 5;
	storeSongService.request.song = {r,E, f4,E, aes4,Q, f4,Ed, aes4,S,c5,Q, a4,Ed, c5,S, e5,HALF};
	storeSong.call(storeSongService);

	storeSongService.request.number = 6;
	storeSongService.request.song = {r,E, f4,E, aes4,Q, f4,Ed, c5,S,a4,Q, f4,Ed, c5,S, a4,HALF};
	storeSong.call(storeSongService);

	playSongService.request.number = 1;
	playSong.call(playSongService);
	ros::Duration(MEASURE_TIME*2.01).sleep();

	playSongService.request.number = 2;
	playSong.call(playSongService);
	ros::Duration(MEASURE_TIME*2.01).sleep();

	playSongService.request.number = 3;
	playSong.call(playSongService);
	ros::Duration(MEASURE_TIME*1.26).sleep();

	playSongService.request.number = 4;
	playSong.call(playSongService);
	ros::Duration(MEASURE_TIME*1.01).sleep();

	playSongService.request.number = 5;
	playSong.call(playSongService);
	ros::Duration(MEASURE_TIME*1.76).sleep();

	playSongService.request.number = 3;
	playSong.call(playSongService);
	ros::Duration(MEASURE_TIME*1.26).sleep();

	playSongService.request.number = 4;
	playSong.call(playSongService);
	ros::Duration(MEASURE_TIME*1.01).sleep();

	playSongService.request.number = 6;
	playSong.call(playSongService);
	ros::Duration(MEASURE_TIME*1.76).sleep();

}