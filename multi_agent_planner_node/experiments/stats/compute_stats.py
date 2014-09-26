#!/usr/bin/python

# import argparse
import os
import csv

# TODO: Get this from the command line arguments
directory_name = '/Users/sid/Documents/research/stats/raw'
NUM_EXP_PER_ENV = 10

class StatsAggregator(object):
    """provides methods to compare stats for the SBPL experiment framework"""

    STATS_FIELDS = ['success_rate']

    @property
    def planner_stats(self):
        return self._planner_stats

    @property
    def planner_name(self):
        return self._planner_name


    def read_from_folder(self, folder_name):
        # get the list of files to read from
        list_of_files = os.listdir(folder_name)
        # save the number of files
        self._num_env = len(list_of_files)
        # flag to store the title row
        header_done = False
        # for each file, go through each row
        for stats_file in list_of_files:
            print 'reading stats from : %s\n' % stats_file
            with open(os.path.join(folder_name, stats_file), 'r') as csvfile:
                reader = csv.reader(csvfile, delimiter=' ')
                for (i, row) in enumerate(reader):
                    if i == 0 and header_done:
                        continue
                    elif i == 0 and not header_done:
                        # read the header and initialize self._planner_stats
                        self._planner_stats = dict.fromkeys(row, 0)
                        header_done = True
                        continue
                    # save raw data.
                    self._raw_data.append(row)
        print 'done reading %s stats' % self._planner_name

    def compute_from_raw_data(self):
        self._stats['success_rate'] = len(self._raw_data)*100 / float(NUM_EXP_PER_ENV *
            self._num_env)

    def print_stats(self):
        for (k,v) in self._stats.items():
            print k, '\t', v
        

    def __init__(self, planner_name):
        super(StatsAggregator, self).__init__()
        self._planner_name = planner_name
        self._num_env = 0
        self._planner_stats = dict()
        self._stats = dict()
        self._raw_data = list()
