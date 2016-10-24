void PointCloudLineConnection::lineConnection() {

    for (auto chunkLineIter = mLines.begin(); chunkLineIter != mLines.end(); ++chunkLineIter) {

        linesType chunkLines = chunkLineIter->second;

        if (mLinesConnected.size() == 0) {
            for (size_t i = 0; i < chunkLines.size(); ++i) {
                linesType lines;
                lines.push_back(chunkLines[i]);
                mLinesConnected[mCurrentGroupN++] = lines;
            }
            continue;
        }

        //compute the distance between lines in one chunk and every last line in the mLinesConnected
        //<<groupid, lineid>, distance>
        std::vector<std::pair<std::pair<groupID, unsigned int>, double>> distances;
        for (size_t i = 0; i < chunkLines.size(); ++i) {
            for (auto iter = mLinesConnected.begin(); iter != mLinesConnected.end(); ++iter) {
                lineType lastLine = (iter->second).back();
                double distance = Line<3>::distanceBetweenTwoLines(*(lastLine.lineModel), *(chunkLines[i].lineModel));
                distances.push_back(std::make_pair(std::make_pair(iter->first, i), distance));
            }
        }

        std::sort(distances.begin(), distances.end(),
                  [](const std::pair<std::pair<unsigned int, unsigned int>, double>& first,const std::pair<std::pair<unsigned int, unsigned int>, double>& second) {
            return first.second < second.second;
        });

        /// Line which has been add to the mLineConnected
        std::set<unsigned int> usedChunkLine;

        /// Group mLineConnected which has had one new line added
        std::set<unsigned int> usedGroup;

        /// Connect a pair of Group and Line if their distance is small within tolerance, and they are not yet connected
        /// with other Line and Group with even smaller distance. If a Line could not find a connectable Group, use it to
        /// start a new Group.
        for (auto iter = distances.begin(); iter != distances.end(); ++iter) {

            /// Calculate the chunk range between current chunk and the last chunk of this group.
            int groupID = iter->first.first;
            int idOfLastChunkOfThisGroup = mLinesConnected[groupID].back().rawLine.first.mChunkID;
            int idOfChunkOfThisLine = chunkLines.back().rawLine.first.mChunkID;

            if (iter->second > mNewLineConnectionThreshold || std::abs(idOfLastChunkOfThisGroup-idOfChunkOfThisLine) > mChunkSearchRangeOfLineConnection) {
                continue;
            }
            if (usedGroup.find((iter->first).first) == usedGroup.end() && usedChunkLine.find((iter->first).second) == usedChunkLine.end()) {
                (mLinesConnected.find((iter->first).first)->second).push_back(chunkLines[(iter->first).second]);
                usedGroup.insert((iter->first).first);
                usedChunkLine.insert((iter->first).second);
            }
        }

        //make a new group for the line which has not been connected
        for (auto iter = distances.begin(); iter != distances.end(); ++iter) {
            if (usedChunkLine.find((iter->first).second) == usedChunkLine.end()) {
                linesType lines;
                lines.push_back(chunkLines[(iter->first).second]);
                mLinesConnected[mCurrentGroupN++] = lines;
                usedChunkLine.insert((iter->first).second);
            }
        }
    }

    //delete the group which has only one line
    for (auto iter = mLinesConnected.begin(); iter != mLinesConnected.end(); ) {
        if ((iter->second).size() == 1) {
            iter = mLinesConnected.erase(iter);
        } else {
            ++iter;
        }
    }
}

