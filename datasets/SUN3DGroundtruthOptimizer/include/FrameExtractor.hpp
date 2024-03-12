#ifndef FRAMEEXTRACTOR_HPP
#define FRAMEEXTRACTOR_HPP

#include <mutex>

#include <Dataloader.hpp>
#include <SIFTextractor.hpp>
#include <ThreadPool.hpp>

void ExtractFrames(std::string local_dir);

#endif // FRAMEEXTRACTOR_HPP