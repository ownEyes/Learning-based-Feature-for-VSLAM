    // This code is a demo to download data from the SUN3D server and load the data in C++.

    #include <pwd.h>
    #include <unistd.h>
    #include <iostream>
    #include <string>
    #include <vector>
    #include <cstdlib>
    #include <curl/curl.h>
    #include <dirent.h>
    #include <cerrno>
    #include <algorithm>
    #include "ThreadPool.h"

    using namespace std;


////////////////////////////////////////////////////////////////////////////////

void SystemCommand(string str) {
    if (system(str.c_str()))
        return;
}

////////////////////////////////////////////////////////////////////////////////

size_t WriteString(char *buf, size_t size, size_t nmemb, string *name) {
    (*name) += string(buf, size * nmemb);
    return size * nmemb;
}

////////////////////////////////////////////////////////////////////////////////

void GetServerFileName(CURL *curl, string *url_name, string *name) {
    cout << "Get file list --> " << endl;

    curl_easy_setopt(curl, CURLOPT_URL, url_name->c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteString);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, name);

    curl_easy_perform(curl);
}

////////////////////////////////////////////////////////////////////////////////

void ParseNames(const string &names, string ext, const string &sun3d_dir, const string &local_dir, vector<string> *file_i, vector<string> *file_o) {
    ext += "\"";

    unsigned int pos = names.find(ext);
    while (pos < names.size()) {
        unsigned p = names.rfind("\"", pos);
        p++;

        string n = names.substr(p, pos - p + 4);
        cout << "-";
        file_i->push_back(sun3d_dir + n);
        file_o->push_back(local_dir + n);

        pos = names.find(ext, pos + ext.size());
    }

    cout << endl << endl;
}

////////////////////////////////////////////////////////////////////////////////

size_t WriteFile(void *ptr, size_t size, size_t nmemb, FILE *stream) {
    return fwrite(ptr, size, nmemb, stream);
}

////////////////////////////////////////////////////////////////////////////////

void DownloadFile(const string &input_url, const string &output_path) {
    CURL *curl = curl_easy_init();
    if (curl) {
        FILE *fp = fopen(output_path.c_str(), "wb");
        if (fp) {
            curl_easy_setopt(curl, CURLOPT_URL, input_url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteFile);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
            curl_easy_perform(curl);
            fclose(fp);
        }
        curl_easy_cleanup(curl);
    }
}

////////////////////////////////////////////////////////////////////////////////

void WriteNames(vector<string> *file_i, vector<string> *file_o) {
    cout << "Copy file list -->" << endl;

    ThreadPool pool(20); // Adjust the number of threads as needed

    for (size_t i = 0; i < file_i->size(); ++i) {
        // Capture the current file_i and file_o values by copy to ensure each lambda has the correct values
        string input_url = (*file_i)[i];
        string output_path = (*file_o)[i];

        cout << output_path << endl;
        pool.enqueue([input_url, output_path] {
            DownloadFile(input_url, output_path);
        });
    }
    // The ThreadPool destructor will automatically wait for all tasks to finish
}


    ////////////////////////////////////////////////////////////////////////////////

    void ServerToLocal( CURL *curl,
        string server_dir, const string &ext, const string &local_dir) {
    vector<string> file_i;
    vector<string> file_o;

    string names;

    GetServerFileName(curl, &server_dir, &names);
    ParseNames(names, ext, server_dir, local_dir, &file_i, &file_o);
    names.clear();
    WriteNames(&file_i, &file_o);
    }

    ////////////////////////////////////////////////////////////////////////////////

    void DataFromServerToLocal(const string &sequence_name,
                            const string &local_dir) {
    string sun3d_path   = "https://sun3d.cs.princeton.edu/data/" + sequence_name;

    string sun3d_camera = sun3d_path + "intrinsics.txt";
    string sun3d_image  = sun3d_path + "image/";
    string sun3d_depth  = sun3d_path + "depth/";
    string sun3d_pose   = sun3d_path + "extrinsics/";

    string local_camera = local_dir  + "intrinsics.txt";
    string local_image  = local_dir  + "image/";
    string local_depth  = local_dir  + "depth/";
    string local_pose   = local_dir  + "extrinsics/";

    string image_ext    = ".jpg";
    string depth_ext    = ".png";
    string pose_ext     = ".txt";

    SystemCommand( "mkdir -p " + local_dir);
    SystemCommand( "mkdir -p " + local_image);
    SystemCommand( "mkdir -p " + local_depth);
    SystemCommand( "mkdir -p " + local_pose);

    CURL* curl;
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();

    ServerToLocal(curl, sun3d_path,  pose_ext,  local_dir);
    ServerToLocal(curl, sun3d_image, image_ext, local_image);
    ServerToLocal(curl, sun3d_depth, depth_ext, local_depth);
    ServerToLocal(curl, sun3d_pose,  pose_ext,  local_pose);

    curl_easy_cleanup(curl);
    curl_global_cleanup();
    }

    ////////////////////////////////////////////////////////////////////////////////

    int main(int argc, char **argv) {
    string sequence_name;
    string local_dir;

    if (argc == 1) {
        passwd* pw = getpwuid(getuid());
        std::string home_dir(pw->pw_dir);
        sequence_name = "harvard_c11/hv_c11_2/";
        local_dir     = home_dir + "/data/sun3d/" + sequence_name;
    } else if (argc == 3) {
        sequence_name = argv[1];
        local_dir     = argv[2];
        local_dir     += sequence_name;
    } else {
        cout << "Server and local directories are needed." << endl;
    }

    DataFromServerToLocal(sequence_name, local_dir);

    return 0;
    }