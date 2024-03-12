#include <iostream>
#include <pwd.h>
#include <unistd.h>

#include <FrameExtractor.hpp>

using namespace std;


int main(int argc, char **argv)
{
    string sequence_name;
    string local_dir;
    if (argc == 1)
    {
        passwd *pw = getpwuid(getuid());
        std::string home_dir(pw->pw_dir);
        sequence_name = "harvard_c11/hv_c11_2/";
        local_dir = home_dir + "/data/sun3d/" + sequence_name;
    }
    else if (argc == 3)
    {
        sequence_name = argv[1];
        local_dir = argv[2];
        local_dir += sequence_name;
    }
    else
    {
        cout << "Server and local directories are needed." << endl;
    }

    ExtractFrames(local_dir);
    return 0;
}
