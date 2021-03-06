
DOXYFILE = 'Doxyfile-mcss'
MAIN_PROJECT_URL: "Pressio"

LINKS_NAVBAR1 = [
  # Get Started
  #("<a href=\"md_pages_get_started.html\">Get Started</a>",
  # ("<a href=>Get Started</a>", # empty href so that get started only has children
  #  [
  #    ("<a href=\"md_pages_getstarted_build_and_install.html\">Installation and Dependencies</a>",)
  #  ]),

  # preparing your app
  ("<a href=\"md_pages_prepare_your_app.html\">How to interface your app</a>",
   []),

  # synopsis
  ("<a>Synopsis</a>",
   [
     ("<a href=\"md_pages_synopsis_galerkin.html\">Galerkin</a>", ),
     ("<a href=\"md_pages_synopsis_lspg.html\">LSPG</a>", ),
     ("<a href=\"md_pages_synopsis_wls.html\">WLS</a>", )
   ]),


  # tutorials
  ("<a>Tutorials</a>", #("<a href=\"md_pages_tutorials.html\">Tutorials</a>",
   [
     ("<a href=\"md_pages_tutorials_tutorial1.html\">Linear Decoder</a>", ),
     ("<a href=\"md_pages_tutorials_tutorial2.html\">Custom Decoder</a>", ),
     ("<a href=\"md_pages_tutorials_tutorial3.html\">Default Galerkin explicit</a>", ),
     ("<a href=\"md_pages_tutorials_tutorial4.html\">Masked Galerkin explicit</a>", )
   ]),

  # demos
  ("<a>Demos</a>", #("<a href=\"md_pages_demos.html\">Demos</a>",
   [
     ("<a href=\"md_pages_demos_demo1.html\">1D adv-diff: Galerkin with POD modes</a>", ),
     ("<a href=\"md_pages_demos_demo2.html\">1D adv-diff: LSPG with POD modes</a>", ),
     ("<a href=\"md_pages_demos_demo3.html\">1D adv-diff: LSPG with nonlinear manifold (kPCA) </a>", ),
     ("<a href=\"md_pages_demos_demo4.html\">1D adv-diff: POD Galerkin with collocation mask</a>", ),
     ("<a href=\"md_pages_demos_demo5.html\">1D adv-diff: Collocation-based Masked POD Galerkin vs LSPG</a>", ),
     ("<a href=\"md_pages_demos_demo6.html\">1D adv-diff: LSPG with nonlinear manifold (MLP) </a>", ),
   ])

  # # hyper-reduction
  # ("<a href=\"md_pages_hyperreduction.html\">Hyper-reduction</a>",
  #  [
  #    ("<a href=\"md_pages_hyperreduction_hyperred_how_to_enable.html\">How to enable hyperreduction</a>",)
  #  ]),

  # # Adapter API
  # ("<a href=\"md_pages_adapter_api.html\">Adapter API</a>",
  #  [
  #    ("<a href=\"md_pages_adapter_apis_adapter_galerkin_api.html\">Galerkin ROM</a>",),
  #    ("<a href=\"md_pages_adapter_apis_adapter_unsteady_lspg_api.html\">Unsteady LSPG ROM</a>",),
  #    ("<a href=\"md_pages_adapter_apis_adapter_discrete_time_api.html\">Discrete-time API</a>",)
  #    #("<a href=\"md_pages_adapter_apis_adapter_steady_lspg_api.html\">Steady LSPG ROM</a>",)
  #  ])
]

LINKS_NAVBAR2 = [
  #('Classes', 'annotated', []),
  #('Namespaces', 'namespaces', [])
]

PLUGINS = ['m.htmlsanity', 'm.math', 'm.code', 'm.components', 'm.dot', 'm.images']

SHOW_UNDOCUMENTED = "YES"

FAVICON = 'favicon.ico'


# STYLESHEETS = [
#     'https://fonts.googleapis.com/css?family=Libre+Baskerville:400,400i,700,700i%7CSource+Code+Pro:400,400i,600',
#     '../css/m-light+documentation.compiled.css'
# ]
# THEME_COLOR = '#91cff4'
# FAVICON = 'favicon-light.png'

# STYLESHEETS = [
#     'https://fonts.googleapis.com/css?family=Source+Sans+Pro:400,400i,600,600i%7CSource+Code+Pro:400,400i,600',
#     '../css/m-dark+documentation.compiled.css'
# ]
# THEME_COLOR = '#cb4b16'
# FAVICON = 'favicon-dark.png'
