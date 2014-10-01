#!/usr/bin/python

# import argparse
import csv
import os

# TODO: Get this from the command line arguments
# directory_name = '/Users/sid/Documents/research/stats/raw'
NUM_EXP_PER_ENV = 10

class TrialStat(object):
    """Data structure to store stats about each trial"""
    IDXMAP = {  'id' : 0,
                'total_planning_time': 1,
                'expansions': 4,
                'solution_cost': 9,
                'path_length' : 10,
                'num_leader_changes' : 11,
                'num_generated_successors' : 12,
                'num_evaluated_successors' : 13 }

    def __getitem__(self, key):
        if not key in TrialStat.IDXMAP.keys():
            raise KeyError("no field called %s" % key)
        data = self._data[TrialStat.IDXMAP[key]]
        if (data == 0.0) and key == 'total_planning_time':
            data += 0.01
        return data

    def __init__(self, raw_data):
        super(TrialStat, self).__init__()
        self._data = raw_data

class EnvStat(object):
    """Aggregation of TrialStats from the current environment"""
    def __init__(self, id, raw_data):
        super(EnvStat, self).__init__()
        self.id = id
        self.raw_data = raw_data
        self.trial_stats = list()
        self.init_from_raw(raw_data)

    @property
    def success_count(self):
        return len(self.trial_stats)

    def get_trial(self, num):
        possible_trial = [trial for trial in self.trial_stats if trial['id'] == num]
        return possible_trial[0] if len(possible_trial) else None

    def compute_on_list(self, name, func):
        values = [trial[name] for trial in self.trial_stats]
        return func(values)

    def __cmp__(self, other):
        return cmp(self.id, other.id)

    def init_from_raw(self, raw_data):
        for trial_data in raw_data:
            self.trial_stats.append(TrialStat(trial_data))

    def count_common_trials(self, other):
        total_common = 0
        for trial in self.trial_stats:
            other_trial = other.get_trial(trial['id'])
            if other_trial is None:
                continue
            total_common += 1
        return total_common

    def compare_with(self, name, other):
        if not isinstance(other, EnvStat):
            raise TypeError("Not an EnvStat object!")
        sum_value_this = 0
        sum_value_other = 0
        total_common = 0
        for trial in self.trial_stats:
            other_trial = other.get_trial(trial['id'])
            if other_trial is None:
                continue
            # ratio = float(trial[name]) / other_trial[name]
            sum_value_this += trial[name]
            sum_value_other += other_trial[name]
            # average_ratio = float(total_common*average_ratio + ratio)/(total_common + 1)
            total_common += 1
        avg_value_this = sum_value_this / total_common
        avg_value_other = sum_value_other / total_common
        return (avg_value_this, avg_value_other, total_common)


class StatsAggregator(object):
    """provides methods to compare stats for the SBPL experiment framework"""

    STATS_FIELDS = ['success_rate']

    @property
    def planner_name(self):
        return self._planner_name

    def get_env(self, num):
        possible_env = [env for env in self._env_stats if env.id == num]
        return possible_env[0] if len(possible_env) else None

    def read_from_folder(self, folder_name):
        # get the list of files to read from
        list_of_files = os.listdir(folder_name)
        # save the number of files
        self._num_env = len(list_of_files)
        # flag to store the title row
        header_done = False
        # for each file, go through each row
        for (e, stats_file) in enumerate(list_of_files):
            # print 'reading stats from : %s' % stats_file
            env_raw_stats = list()
            with open(os.path.join(folder_name, stats_file), 'r') as csvfile:
                reader = csv.reader(csvfile, delimiter=' ')
                for (i, row) in enumerate(reader):
                    if i == 0:
                        continue
                    # cleanup
                    row = [r for r in row if r]
                    # convert to float
                    row = [float(r) for r in row]
                    # create a list of the stat lines
                    env_raw_stats.append(row)
            env_num = re.findall('\d+', stats_file)[0]
            self._env_stats.append(EnvStat(env_num, env_raw_stats))
        self._env_stats.sort()
        self.compute_from_raw_data()
        print 'done reading %s stats' % self._planner_name

    def compute_from_raw_data(self):
        success_count = sum([env_stat.success_count for env_stat in self._env_stats])
        self._stats['success_rate'] = success_count*100 / float(NUM_EXP_PER_ENV*self._num_env)

        total_planning_time = sum([env_stat.compute_on_list('total_planning_time', sum) for env_stat in self._env_stats])
        self._stats['avg_planning_time'] = total_planning_time / success_count

        num_evaluated_successors = sum([env_stat.compute_on_list('num_evaluated_successors', sum) for env_stat in self._env_stats])
        num_generated_successors = sum([env_stat.compute_on_list('num_generated_successors', sum) for env_stat in self._env_stats])
        self._stats['evaluated_to_generated'] = num_evaluated_successors/num_generated_successors

    def print_stats(self):
        print 'Planner : %s' % self.planner_name
        for (k,v) in self._stats.items():
            print k, '\t', v
        print '-----'

    def count_common_trials(self, other):
        total_common = 0
        for env in self._env_stats:
            other_env = other.get_env(env.id)
            if other_env is None:
                continue
            total_common += env.count_common_trials(other_env)
        return total_common

    def aggregate_env_stat(self, name, other_stats_aggr):
        if not isinstance(other_stats_aggr, StatsAggregator):
            raise TypeError("Invalid type!")
        total_common = 0
        avg_value_this = float(0)
        avg_value_other = float(0)
        for env in self._env_stats:
            other_env = other_stats_aggr.get_env(env.id)
            if other_env is None:
                continue
            (avg_this, avg_other, num_common) = env.compare_with(name, other_env)
            # average_ratio = float(total_common*average_ratio + ratio*num_common)/(total_common + num_common)
            avg_value_this = (avg_value_this*total_common + avg_this*num_common) / (total_common + num_common)
            avg_value_other = (avg_value_other*total_common + avg_other*num_common) / (total_common + num_common)
            total_common += num_common
        return (avg_value_this, avg_value_other, total_common)


    def compare_with(self, other):
        """Nomenclature : self.compare_with(other) sets other to 1 and gets the ratios"""
        if not isinstance(other, StatsAggregator):
            raise TypeError("comparison object should be another StatsAggregator!")
        self.print_stats()
        other.print_stats()
        
        print 'Comparing %s with %s' % (self.planner_name, other.planner_name)
        print '-----'
        print 'Number of common trials: %d' % self.count_common_trials(other)
        # get the planning time average ratio
        (avg_planning_time_this, avg_planning_time_other, num_common) = self.aggregate_env_stat('total_planning_time', other)
        print 'Average planning time (%s/%s):\t %0.2f & %0.2f' %(self.planner_name, other.planner_name, avg_planning_time_this, avg_planning_time_other)

        # get the expansions average ratio
        (avg_expansions_this, avg_expansions_other, num_common) = self.aggregate_env_stat('expansions', other)
        print 'Average expansions (%s/%s):\t %0.2f & %0.2f' %(self.planner_name, other.planner_name, avg_expansions_this, avg_expansions_other)

        # get the solution_cost average ratio
        (avg_solution_cost_this, avg_solution_cost_other, num_common) = self.aggregate_env_stat('solution_cost', other)
        print 'Average solution_cost (%s/%s):\t %0.2f & %0.2f' %(self.planner_name, other.planner_name, avg_solution_cost_this, avg_solution_cost_other)

        # get the path length average ratio
        (avg_path_length_this, avg_path_length_other, num_common) = self.aggregate_env_stat('path_length', other)
        print 'Average path_length (%s/%s):\t %0.2f & %0.2f' %(self.planner_name, other.planner_name, avg_path_length_this, avg_path_length_other)

        # get the num leader change average ratio
        (avg_num_leader_changes_this, avg_num_leader_changes_other, num_common) = self.aggregate_env_stat('num_leader_changes', other)
        print 'Average num_leader_changes (%s/%s):\t %0.2f & %0.2f' %(self.planner_name, other.planner_name, avg_num_leader_changes_this, avg_num_leader_changes_other)

        # get the num generated successors average ratio
        (avg_num_generated_successors_this, avg_num_generated_successors_other, num_common) = self.aggregate_env_stat('num_generated_successors', other)
        print 'Average num_generated_successors (%s/%s):\t %0.2f & %0.2f' %(self.planner_name, other.planner_name, avg_num_generated_successors_this, avg_num_generated_successors_other)

        # get the num evaluated successors average ratio
        (avg_num_evaluated_successors_this, avg_num_evaluated_successors_other, num_common) = self.aggregate_env_stat('num_evaluated_successors', other)
        print 'Average num_evaluated_successors (%s/%s):\t %0.2f & %0.2f' %(self.planner_name, other.planner_name, avg_num_evaluated_successors_this, avg_num_evaluated_successors_other)

    def __init__(self, planner_name):
        super(StatsAggregator, self).__init__()
        self._planner_name = planner_name
        self._num_env = 0
        self._stats = dict()
        self._env_stats = list()