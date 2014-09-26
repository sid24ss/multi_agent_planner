#pragma once

#include <vector>
#include <cstdio>
using namespace std;

class StatsWriter{
  public:
    static void writeStatsToFile(string filename,
                                bool start_new_file,
                                vector<string> names,
                                vector<double> values,
                                int s_no)
    {
        FILE* fout;
        if(start_new_file){
            fout = fopen(filename.c_str(),"w");
            fprintf(fout, "#s_no ");
            for(unsigned int i=0; i<names.size(); i++)
              fprintf(fout, "%s ", names[i].c_str());
            fprintf(fout, "\n");
        }
        else
            fout = fopen(filename.c_str(),"a");

        fprintf(fout, "%d ", s_no);
        for(unsigned int i=0; i<values.size(); i++)
            fprintf(fout, "%.2f ", values[i]);

        fprintf(fout, "\n");
        fclose(fout);
    };
};
