#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <filesystem>
#include <opencv2/features2d.hpp>
#include "Inference.hpp"

namespace fs = std::filesystem;

std::vector<std::tuple<int, int, float>>
pairwise_matches(const Eigen::MatrixXf &last_keypoints,
                 const Eigen::MatrixXf &next_keypoints)
{
    // store correspondences (last_id, next_id)
    std::vector<std::tuple<int, int, float>> match_ids;

    if (last_keypoints.rows() > 0)
    {
        cv::Mat last_descr;
        cv::eigen2cv(Eigen::MatrixXf(last_keypoints.rightCols(last_keypoints.cols())), last_descr);
        cv::Mat next_descr;
        cv::eigen2cv(Eigen::MatrixXf(next_keypoints.rightCols(next_keypoints.cols())), next_descr);

        // match descriptor pairs (query=next, train=last)
        std::vector<cv::DMatch> matches;
        cv::BFMatcher(cv::NORM_L2, true).match(next_descr, last_descr, matches);

        for (const cv::DMatch &match : matches)
        {
            
            if (match.distance > 0.4)
                continue;
            
            // std::cout<<"distance:"<<match.distance<<std::endl;
            match_ids.push_back(std::make_tuple(match.trainIdx, match.queryIdx, match.distance));
        }
    }
    std::cout << "Total matches: " << match_ids.size() << std::endl;

    return match_ids;
}

std::vector<std::tuple<int, int, int>>
pairwise_matches_binary(const Matrix8u &last_keypoints,
                 const Matrix8u &next_keypoints)
{
    // store correspondences (last_id, next_id)
    std::vector<std::tuple<int, int, int>> match_ids;

    if (last_keypoints.rows() > 0)
    {
        cv::Mat last_descr;
        cv::eigen2cv(Matrix8u(last_keypoints.rightCols(last_keypoints.cols())), last_descr);
        cv::Mat next_descr;
        cv::eigen2cv(Matrix8u(next_keypoints.rightCols(next_keypoints.cols())), next_descr);

        last_descr.convertTo(last_descr, CV_8UC1);
        next_descr.convertTo(next_descr, CV_8UC1);

        cv::Mat packedlast_descr(last_descr.rows, 32, CV_8UC1); // Create a new matrix with 32 columns

        for (int i = 0; i < last_descr.rows; i++) {
            for (int j = 0; j < 32; j++) {
                uint8_t byte = 0;
                for (int bit = 0; bit < 8; bit++) {
                    byte |= (last_descr.at<uint8_t>(i, j * 8 + bit) << bit);
                }
                packedlast_descr.at<uint8_t>(i, j) = byte;
            }
        }

        cv::Mat packednext_descr(next_descr.rows, 32, CV_8UC1); // Create a new matrix with 32 columns

        for (int i = 0; i < next_descr.rows; i++) {
            for (int j = 0; j < 32; j++) {
                uint8_t byte = 0;
                for (int bit = 0; bit < 8; bit++) {
                    byte |= (next_descr.at<uint8_t>(i, j * 8 + bit) << bit);
                }
                packednext_descr.at<uint8_t>(i, j) = byte;
            }
        }

        // match descriptor pairs (query=next, train=last)
        std::vector<cv::DMatch> matches;
        // cv::BFMatcher(cv::NORM_L2, true).match(next_descr, last_descr, matches);
        cv::BFMatcher(cv::NORM_HAMMING, true).match(packednext_descr, packedlast_descr, matches);

        for (const cv::DMatch &match : matches)
        {
            
            if (match.distance > 128)
                continue;
            
            // std::cout<<"distance:"<<match.distance<<std::endl;
            match_ids.push_back(std::make_tuple(match.trainIdx, match.queryIdx, match.distance));
        }
    }
    std::cout << "Total matches: " << match_ids.size() << std::endl;

    return match_ids;
}


cv::Mat draw_matches(const Eigen::MatrixXf &last_keypoints,
                     const Eigen::MatrixXf &next_keypoints,
                     const std::vector<std::tuple<int, int, int>> &correspondences,
                     const cv::Mat &last_img, const cv::Mat &next_img)
{
    std::vector<cv::DMatch> matches;
    for (const auto &match : correspondences)
        matches.emplace_back(std::get<1>(match), std::get<0>(match), std::get<2>(match));

    std::vector<cv::KeyPoint> last_kp(last_keypoints.rows());
    for (size_t i = 0; i < last_kp.size(); i++)
        last_kp[i].pt = cv::Point(last_keypoints.row(i)[0] , last_keypoints.row(i)[1]);

    std::vector<cv::KeyPoint> next_kp(next_keypoints.rows());
    for (size_t i = 0; i < next_kp.size(); i++)
        next_kp[i].pt = cv::Point(next_keypoints.row(i)[0] , next_keypoints.row(i)[1]);

    cv::Mat img_matches;
    cv::drawMatches(next_img, next_kp, last_img, last_kp, matches, img_matches,
                    cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    return img_matches;
}

cv::Mat draw_tracks(const Eigen::MatrixXf &last_keypoints,
                    const Eigen::MatrixXf &next_keypoints,
                    const std::vector<std::tuple<int, int, int>> &correspondences,
                    const cv::Mat &next_img)
{
    std::vector<cv::Point> last_kp(last_keypoints.rows());
    for (size_t i = 0; i < last_kp.size(); i++)
        last_kp[i] = cv::Point(last_keypoints.row(i)[0], last_keypoints.row(i)[1] );

    std::vector<cv::Point> next_kp(next_keypoints.rows());
    for (size_t i = 0; i < next_kp.size(); i++)
        next_kp[i] = cv::Point(next_keypoints.row(i)[0], next_keypoints.row(i)[1]);

    cv::Mat img_tracks;
    cv::cvtColor(next_img, img_tracks, cv::COLOR_RGB2GRAY);
    cv::cvtColor(img_tracks, img_tracks, cv::COLOR_GRAY2RGB);

    std::uniform_real_distribution<double> u(0, 1);
    std::default_random_engine g;

    for (const auto &[last_id, next_id, distance] : correspondences)
    {
        // sample random colour with track-specific seed
        g.seed(last_id);
        // draw track segments
        cv::line(img_tracks, last_kp[last_id], next_kp[next_id], {u(g) * 255, u(g) * 255, u(g) * 255}, 2);
    }

    return img_tracks;
}

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "wrong set of arguments, expected:" << std::endl
                  << argv[0] << " <model_path> <image_1_path> ... <image_N_path>" << std::endl;
        return EXIT_FAILURE;
    }

    // read command line arguments: <model_path> <image_1_path> ... <image_N_path>
    const std::string keypoint_predictor_path = argv[1];

    // load trained model
    const Extractor sp(keypoint_predictor_path);

    // load images
    std::vector<cv::Mat> img;
    for (int i = 0; i < argc - 2; i++)
    {
        img.push_back(cv::imread(argv[2 + i]));
    }

    std::cout << "images: " << img.size() << std::endl;

    std::vector<Eigen::MatrixX2d> coord(img.size());
    //std::vector<Eigen::MatrixXd> descr(img.size());
    std::vector<Matrix8u> descr(img.size());
    for (int i = 0; i < img.size(); i++)
    {
        cv::Mat rgb;
        cv::cvtColor(img[i], rgb, cv::COLOR_BGR2RGB);

        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints;
        orb->detect(rgb, keypoints);
        // // convert to RGB for inference
        std::vector<cv::KeyPoint> valid_keypoints;
        std::tie(coord[i], descr[i],valid_keypoints) = sp.getFeatures(rgb,keypoints);

        if (i > 0)
        {
            // draw & export matches
            //const auto matches = pairwise_matches(descr[i - 1].cast<float>(), descr[i].cast<float>());
            const auto matches = pairwise_matches_binary(descr[i - 1].cast<uint8_t>(), descr[i].cast<uint8_t>());
            const cv::Mat img_matches = draw_matches(coord[i - 1].cast<float>(), coord[i].cast<float>(), matches, img[i - 1], img[i]);
            const std::string match_name = std::to_string(i - 1) + "-" + std::to_string(i);
            cv::imshow("matches " + match_name, img_matches);
            cv::imwrite(fs::temp_directory_path() / fs::path("matches-" + match_name + ".png"), img_matches);

            // draw & export tracks
            const cv::Mat img_tracks = draw_tracks(coord[i - 1].cast<float>(), coord[i].cast<float>(), matches, img[i]);
            cv::imshow("tracks " + match_name, img_tracks);
            cv::imwrite(fs::temp_directory_path() / fs::path("tracks-" + match_name + ".png"), img_tracks);
        }
    }
    cv::waitKey();

    return EXIT_SUCCESS;
}