## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    obj = bld.create_ns3_program('vanet-test',
        ['core', 'aodv', 'applications', 'dsr', 'dsdv', 'flow-monitor', 'mobility', 'network', 'olsr', 'propagation', 'wifi', 'wave'])
    obj.source = 'vanet-test.cc'