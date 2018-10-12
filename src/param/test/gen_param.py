import os
import json
import glob

def files(curr_dir = '.', ext = '*.exe'):
    """curr file"""
    for i in glob.glob(os.path.join(curr_dir, ext)):
        yield i
        
def generate_api_c(directory, param):
    '''api.c'''
    f = open(os.path.join(directory, "param_api.c"), mode='w')
    
    str='''#include <stdint.h>
#include "param.h"
#include "param_api.h"   

extern struct param_val param_list[] = {
'''    
    f.write(str)

    for i in range(len(param)):    
        param_group = param.keys()[i]
        print param[param_group]
        for param_name in param[param_group]:    
            param_name_low = param_name.lower()
            str='''\t{"%s", param_get_%s, param_set_%s},\n''' %(param_name,param_name_low,param_name_low)
            f.write(str)
        f.write("\n")    

    str='''};\n\n'''
    f.write(str)
    
    for group_name in param:    
        str='''extern struct %s_param_s* %s_param;\n''' % (group_name.lower(),group_name.lower())
        f.write(str)

    f.close()
    
def generate_api_h(directory, param):
    '''api.h'''
    
    param_len=0;
    
    f = open(os.path.join(directory, "param_api.h"), mode='w')
    
    str='''#pragma once

#include "param.h"

extern struct param_val param_list[];\n''' 
    f.write(str)

        
    # for i in range(len(param)):    
        # param_group = param.keys()[i]
        # str='''\nvoid param_register_%s(void* param);\n''' % param_group.lower()
        # f.write(str)
        # for param_name in param[param_group]:   
            # param_len += 1
            # str='''float param_get_%s(void);\n''' %param_name.lower()
            # f.write(str)
            # str='''void param_set_%s(float v);\n''' %param_name.lower()
            # f.write(str)
   
    for i in range(len(param)):    
        param_group = param.keys()[i]
        param_len += len(param[param_group])
    str='''\n#define PARAM_LIST_MAX %d\n\n''' % param_len
    f.write(str)
    
    for group_name in param:       
        str='''#include "param_gen_%s.h"\n''' %group_name.lower()
        f.write(str)    
    
    f.close()    
 

def generate_param_group_h(directory, group, param):
    '''param_group.h'''
    
    group_low = group.lower()
    
    f = open(os.path.join(directory, "param_gen_"+group_low+".h"), mode='w')
    str = '''#include <stdint.h>
#include "param.h"
#include "param_api.h"\n\n'''
    f.write(str)

    str='''struct %s_param_s {\n''' %group_low 
    f.write(str)

    for param_name in param:  
        str='''\tfloat %s;\n''' %param_name.lower()
        f.write(str)

    str='''};\n\n'''
    f.write(str)

    str='''static inline void param_register_%s(void* param)
{
	%s_param = (struct %s_param_s*)param;
}\n\n''' %(group_low,group_low,group_low)
    f.write(str)


    for param_name in param: 
        param_name_low = param_name.lower()
        str='''static inline float param_get_%s(void)
{
    return %s_param->%s;
}

static inline void param_set_%s(float v)
{
    %s_param->%s = v;
}\n\n''' %(param_name_low,group_low,param_name_low,param_name_low,group_low,param_name_low)
        f.write(str)

    f.close()    
    
    

j = open("ff.param", mode='r')   

param = json.load(j)
print param

print len(param)
  
  
for i in files(".", "param_gen_*.h"):
    print i
    os.remove(i)  
  
generate_api_c(".", param)
generate_api_h(".", param)
for i in range(len(param)):    
    param_group = param.keys()[i]
    generate_param_group_h(".", param_group, param[param_group])

  