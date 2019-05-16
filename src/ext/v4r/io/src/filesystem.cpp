#include "filesystem.h"

#include <fstream>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

namespace v4r {
namespace io {

std::vector<std::string> getFoldersInDirectory(const bf::path &dir) {
  std::vector<std::string> relative_paths;

  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr(dir); itr != end_itr; ++itr) {
    if (bf::is_directory(*itr))
      relative_paths.push_back(itr->path().filename().string());
  }
  std::sort(relative_paths.begin(), relative_paths.end());

  return relative_paths;
}

std::vector<std::string> getFilesInDirectory(const bf::path &dir, const std::string &regex_pattern, bool recursive) {
  std::vector<std::string> relative_paths;

  if (!v4r::io::existsFolder(dir) || !bf::is_directory(dir)) {
    std::cerr << dir << " is not a directory!" << std::endl;
  } else {
    bf::path bf_dir = dir;
    bf::directory_iterator end_itr;
    for (bf::directory_iterator itr(bf_dir); itr != end_itr; ++itr) {
      const bf::path file = itr->path().filename();

      // check if its a directory, then get files in it
      if (bf::is_directory(*itr)) {
        if (recursive) {
          bf::path fn = dir / file;
          std::vector<std::string> files_in_subfolder = getFilesInDirectory(fn.string(), regex_pattern, recursive);
          for (const auto &sub_file : files_in_subfolder) {
            bf::path sub_fn = file / sub_file;
            relative_paths.push_back(sub_fn.string());
          }
        }
      } else {
        // check for correct file pattern (extension,..) and then add, otherwise ignore..
        boost::smatch what;
        const boost::regex file_filter(regex_pattern);
        if (boost::regex_match(file.string(), what, file_filter))
          relative_paths.push_back(file.string());
      }
    }
    std::sort(relative_paths.begin(), relative_paths.end());
  }
  return relative_paths;
}

bool existsFile(const bf::path &rFile) {
  return (bf::exists(rFile) && bf::is_regular_file(rFile));
}

bool existsFolder(const bf::path &dir) {
  return bf::exists(dir);
}

void createDirIfNotExist(const bf::path &dir) {
  if (!bf::exists(dir))
    bf::create_directories(dir);
}

void createDirForFileIfNotExist(const bf::path &filename) {
  createDirIfNotExist(filename.parent_path());
}

void copyDir(const bf::path &sourceDir, const bf::path &destinationDir) {
  if (!bf::exists(sourceDir) || !bf::is_directory(sourceDir)) {
    throw std::runtime_error("Source directory " + sourceDir.string() + " does not exist or is not a directory");
  }
  if (bf::exists(destinationDir)) {
    throw std::runtime_error("Destination directory " + destinationDir.string() + " already exists");
  }
  if (!bf::create_directory(destinationDir)) {
    throw std::runtime_error("Cannot create destination directory " + destinationDir.string());
  }

  typedef bf::recursive_directory_iterator RDIter;
  for (auto it = RDIter(sourceDir), end = RDIter(); it != end; ++it) {
    const auto &iteratorPath = it->path();
    auto relativeIteratorPathString = iteratorPath.string();
    boost::replace_first(relativeIteratorPathString, sourceDir.string(), "");

    bf::copy(iteratorPath, destinationDir / relativeIteratorPathString);
  }
}

void removeDir(const bf::path &path) {
  if (v4r::io::existsFolder(path)) {
    for (bf::directory_iterator end_dir_it, it(path); it != end_dir_it; ++it)
      bf::remove_all(it->path());

    bf::remove(path);
  } else
    std::cerr << "Folder " << path.string() << " does not exist." << std::endl;
}

bf::path getDataDir() {
  bf::path p;
  if (auto value = std::getenv("XDG_DATA_HOME"))
    p = value;
  else if (auto value = std::getenv("HOME"))
    p = bf::path(value) / ".local" / "share";
  else
    throw std::runtime_error("Cannot construct data directory path");
  return p / "v4r";
}

bf::path getConfigDir() {
  bf::path p;
  if (auto value = std::getenv("XDG_CONFIG_HOME"))
    p = value;
  else if (auto value = std::getenv("HOME"))
    p = bf::path(value) / ".config";
  else
    throw std::runtime_error("Cannot construct config directory path");
  return p / "v4r";
}

}  // namespace io
}  // namespace v4r
