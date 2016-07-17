#ifndef PLAY_SONG_LIB
#define PLAY_SONG_LIB

#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/StoreSong.h"

class PlaySongLib
{
    public:
        PlaySongLib()
        {
            storeSong = n.serviceClient<create_fundamentals::StoreSong>("store_song");
            playSong  = n.serviceClient<create_fundamentals::PlaySong>("play_song");
        }

    void starWars();
    void starWarsShort();
    void failure();
    void beep();

    private:
        ros::NodeHandle n;
        
        ros::ServiceClient storeSong;
        ros::ServiceClient playSong;

        create_fundamentals::StoreSong srv;
        create_fundamentals::StoreSong storeSongService;
        create_fundamentals::PlaySong playSongService;
        void storeStarWars();

        // define silence
        static const unsigned int r       = 30;
        
        // map note names in the lilypad notation to irobot commands
        static const unsigned int c4      = 60;
        static const unsigned int cis4    = 61;
        static const unsigned int des4    = cis4;
        static const unsigned int d4      = 62;
        static const unsigned int dis4    = 63;
        static const unsigned int ees4    = dis4;
        static const unsigned int e4      = 64;
        static const unsigned int f4      = 65;
        static const unsigned int fis4    = 66;
        static const unsigned int ges4    = fis4;
        static const unsigned int g4      = 67;
        static const unsigned int gis4    = 68;
        static const unsigned int aes4    = gis4;
        static const unsigned int a4      = 69;
        static const unsigned int ais4    = 70;
        static const unsigned int bes4    = ais4;
        static const unsigned int b4      = 71;
        static const unsigned int c5      = 72;
        static const unsigned int cis5    = 73;
        static const unsigned int des5    = cis5;
        static const unsigned int d5      = 74;
        static const unsigned int dis5    = 75;
        static const unsigned int ees5    = dis5;
        static const unsigned int e5      = 76;
        static const unsigned int f5      = 77;
        static const unsigned int fis5    = 78;
        static const unsigned int ges5    = fis5;
        static const unsigned int g5      = 79;
        static const unsigned int gis5    = 80;
        static const unsigned int aes5    = gis5;
        static const unsigned int a5      = 81;
        static const unsigned int ais5    = 82;
        static const unsigned int bes5    = ais5;
        static const unsigned int b5      = 83;
        static const unsigned int c6      = 84;
        static const unsigned int cis6    = 85;
        static const unsigned int des6    = cis6;
        static const unsigned int d6      = 86;
        static const unsigned int dis6    = 87;
        static const unsigned int ees6    = dis6;
        static const unsigned int e6      = 88;
        static const unsigned int f6      = 89;
        static const unsigned int fis6    = 90;
        static const unsigned int ges6    = fis6;
        
        // define some note lengths
        // change the top MEASURE (4/4 time) to get faster/slower speeds
        static const unsigned int MEASURE = 160;
        static const unsigned int HALF    = MEASURE/2;
        static const unsigned int Q       = MEASURE/4;
        static const unsigned int E       = MEASURE/8;
        static const unsigned int Ed      = MEASURE*3/16;
        static const unsigned int S       = MEASURE/16;
        
        float MEASURE_TIME                = (float)(MEASURE)/64;
};

void PlaySongLib::beep()
{
    storeSongService.request.number = 1;
    storeSongService.request.song = {a4,Q,HALF};
    storeSong.call(storeSongService);

    playSongService.request.number = 1;
    playSong.call(playSongService);
}

void PlaySongLib::failure()
{
    storeSongService.request.number = 6;
    storeSongService.request.song = {r,E, f4,E, aes4,Q};
    storeSong.call(storeSongService);

    playSongService.request.number = 6;
    playSong.call(playSongService);
}

void PlaySongLib::storeStarWars()
{
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
}

void PlaySongLib::starWarsShort() 
{
    storeStarWars();

    playSongService.request.number = 1;
    playSong.call(playSongService);
    ros::Duration(MEASURE_TIME*2.01).sleep();
}

void PlaySongLib::starWars() 
{
    storeStarWars();

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
}

#endif // PLAY_SONG_LIB
