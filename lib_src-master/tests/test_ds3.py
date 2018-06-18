import xmostest

def runtest():
    resources = xmostest.request_resource("xsim")

    tester = xmostest.ComparisonTester(open('ds3_test.expect'),
                                       'lib_src', 'fixed_factor_of_3_tests',
                                       'app_ds3', {})

    xmostest.run_on_simulator(resources['xsim'],
                              '../examples/app_ds3/bin/app_ds3.xe',
                              tester=tester)
