#include <play_song.cpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_playsong");
  
  PlaySongLib play_song;
  play_song.beep();

  return 0;
}
