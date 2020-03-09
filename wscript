# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

# def configure(conf):
#     conf.check_nonfatal(header_name='stdint.h', define_name='HAVE_STDINT_H')

def build(bld):
    module = bld.create_ns3_module('hungarian', ['core'])
    module.source = [
        'model/hungarian.cc',
        'model/disconnected.cc',
        'model/agent.cc',
        'model/globalInfo.cc',
        'model/debuggingFunctions.cc',
        'model/messageHandling.cc',
        'helper/hungarian-helper.cc',
        ]

    module_test = bld.create_ns3_module_test_library('hungarian')
    module_test.source = [
        'test/hungarian-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'hungarian'
    headers.source = [
        'model/hungarian.h',
        'model/disconnected.h',
        'model/agent.h',
        'model/structs.h',
        'model/globalInfo.h',
        'model/debuggingFunctions.h',
        'model/messageHandling.h',
        'helper/hungarian-helper.h',
        ]

    if bld.env.ENABLE_EXAMPLES:
        bld.recurse('examples')

    # bld.ns3_python_bindings()

