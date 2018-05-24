import xmostest

def runtest():
    resources = xmostest.request_resource("xsim")

    tester = xmostest.ComparisonTester(open('us3_voice_test.expect'),
                                       'lib_src', 'fixed_factor_of_3_tests',
                                       'app_us3_voice', {})

    xmostest.run_on_simulator(resources['xsim'],
                              '../examples/app_us3_voice/bin/app_us3_voice.xe',
                              tester=tester)
