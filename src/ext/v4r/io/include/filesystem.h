/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file filesystem.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#pragma once

#include <string>
#include <vector>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

namespace bf = boost::filesystem;

namespace v4r {
namespace io {

/** Returns folder names in a folder </br>
 * @param dir
 * @return relative_paths
 */
std::vector<std::string> getFoldersInDirectory(const boost::filesystem::path &dir);

/** Returns a the name of files in a folder </br>
 * '(.*)bmp'
 * @param dir
 * @param regex_pattern examples "(.*)bmp",  "(.*)$"
 * @param recursive (true if files in subfolders should be returned as well)
 * @return files in folder
 */
std::vector<std::string> getFilesInDirectory(const boost::filesystem::path &dir,
                                             const std::string &regex_pattern = std::string(""), bool recursive = true);

/** checks if a file exists
 * @param rFile
 * @return true if file exists
 */
bool existsFile(const boost::filesystem::path &rFile);

/** checks if a folder exists
 * @param rFolder
 * @return true if folder exists
 */
bool existsFolder(const boost::filesystem::path &dir);

/** checks if folder already exists and if not, creates one
 * @param folder_name
 */
void createDirIfNotExist(const boost::filesystem::path &dir);

/** checks if the path for the filename already exists,
 * otherwise creates it
 * @param filename
 */
void createDirForFileIfNotExist(const boost::filesystem::path &filename);

/** @brief copies a directory from source to destination
 * @param path of source directory
 * @param path of destination directory
 */
void copyDir(const bf::path &sourceDir, const bf::path &destinationDir);

/**
 * @brief removeDir remove a directory with all its contents (including subdirectories) from disk
 * @param path folder path
 */
void removeDir(const bf::path &path);

/// Get base directory relative to which user specific V4R data files should be stored.
/// Follows XDG Base Directory Specification:
///   https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html
bf::path getDataDir();

/// Get base directory relative to which user specific V4R configuration files should be stored.
/// Follows XDG Base Directory Specification:
///   https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html
bf::path getConfigDir();

}  // namespace io
}  // namespace v4r
