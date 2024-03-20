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
        sequence_name = "brown_bm_1/brown_bm_1/";
        local_dir = home_dir + "/projects/Learning-based-Feature-for-VSLAM/datasets/sun3d/" + sequence_name;
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
