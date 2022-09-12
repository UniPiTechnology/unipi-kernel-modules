#!/bin/bash

SCRIPT_PATH=$(dirname "$0")
#include some global variables (DEBIAN_VERSION, codenames,...)
[ -r "$SCRIPT_PATH/include.sh" ] && . "$SCRIPT_PATH/include.sh"

# where to generate the Gitlab CI definition
ci_file="${ci_file:-"my-gitlab-ci.yml"}"
package_name="${package_name:=$CI_PROJECT_NAME}"

# complete list of products
products="unipi1 unipi1x64 neuron neuron64 axon g1 zulu patron iris"
additional_products="unipi1u unipi1x64u neuronu neuron64u"

# complete list of architectures
architectures="armhf arm64 amd64"

architecture_unipi1=armhf
architecture_unipi1x64=arm64
architecture_neuron=armhf
architecture_neuron64=arm64
architecture_axon=arm64
architecture_g1=arm64
architecture_zulu=arm64
architecture_patron=arm64
architecture_iris=arm64

architecture_unipi1u=armhf
architecture_unipi1x64u=arm64
architecture_neuronu=armhf
architecture_neuron64u=arm64

# complete list of distributions
# codenames are taken from /ci-scripts/include.sh
distributions="stable oldstable newstable"

# list of excluded product-based building jobs, which are by design non-sense
# those combinations include products which didn't exist when the given distribution was
# active, or are no longer build for newer distribution
# THE DISTRIBUTION CODENAMES ARE USED, INSTEAD OF ALIASES
product_build_jobs_exclude="stretch-unipi1 stretch-unipi1x64 stretch-neuron64 stretch-g1 stretch-zulu stretch-patron stretch-iris buster-unipi1 buster-unipi1x64 buster-neuron64 buster-patron buster-iris"

# this will hold list of actual build jobs (combination of distribution and element (see below))
build_jobs=""

# this will hold build_elements - the second dimension of the build (list of selected products OR architectures)
build_elements=""

# -- DEPRECATED, replaced by build_independent_for -- generate special job for common packages (dkms and unipi-firmware)
if [ "$build_common" = "yes" ] || [ "$build_common" == "true" ]; then
    [ -z "$build_independent_for" ] && build_independent_for="all"
fi

# generate special job for mirroring the repo to GitHub
# we cannot limit this only for main/master branch, because kernels branches
# are named by the version of the kernel, so better to limit it to presence of the tag
# so, mirror if mirroring enabled and the commit is tagged or build is disabled (for repos which are just mirrored, not built. E.g. linux-imx)
if [[ ( "$CI_COMMIT_TAG" != "" || "$disable_build" == "yes" || "$disable_build" == "true" ) && ( "$mirror_to_github" == "yes"  || "$mirror_to_github" == "true" ) ]]; then
    mirror_to_github="true"
fi

# decide in which way to generate the list of the jobs

# sometimes, we don't want to build anything, just mirror to github for example
if [ "$disable_build" = "yes" ] || [ "$disable_build" = "yes" ]; then
    disable_build="true"
elif [ "$build_exactly_for" != "" ]; then
    # the build is started exactly for these jobs
    # this has precedence over everything, no further filtering or checks are applied
    filtered_build_jobs="$build_exactly_for"
else
    # the build should be either for product or architecture
    if [ "$build_for_product" != "" ]; then
        # building for selected products
        if [ "$build_for_product" == "all" ]; then
            # building for all products
            build_elements="$products"
        elif [ "$build_for_product" == "all+" ]; then
            # building for selected products
            build_elements="$products $additional_products"
        else
            build_elements="$build_for_product"
        fi
    elif [ "$build_for_architecture" != "" ]; then
        # building for selected architectures
        if [ "$build_for_architecture" == "all" ]; then
            # building for all architectures
            build_elements="$architectures"
        else
            # building for selected architectures
            build_elements="$build_for_architecture"
        fi
    elif [ "$build_independent_for" = "" ]; then
        # neither build_for_product or build_for_architecture has been chosen
        echo "Unspecified building scheme, expecting either 'build_for_product' or 'build_for_architecture' to be set with relevant data."
        exit 1
    fi

    if [ "$build_independent_for" == "all" ]; then
        # replace space by + in $distributions
        independent_build_jobs="$(echo "$distributions" | sed 's/^\s\+//;s/\s\+$//;s/\s\+/+/g')"
    else
        independent_build_jobs="$build_independent_for"
    fi

    # now decide for which distributions the build will be
    build_distributions=""
    if [ "$build_for_distribution" == "" ] || [ "$build_for_distribution" == "all" ]; then
        # building for all distributions
        build_distributions="$distributions"
    else
        # building for selected distributions
        build_distributions="$build_for_distribution"
    fi

    # generate the list of jobs, comprising of combination of each selected distribution and build element
    for distribution in $build_distributions; do
        for element in $build_elements; do
            # construct the job name and add it to the list
            build_jobs+="$distribution-$element "
        done
    done

    # filter out the user suppressed jobs (the ones with oldstable/stable/newstable in the name)
    filtered_build_jobs=$build_jobs
    if [ "$disable_for" != "" ]; then
        for unwanted in $disable_for; do
            # https://stackoverflow.com/questions/1032023/sed-whole-word-search-and-replace
            filtered_build_jobs="$( echo "$filtered_build_jobs" | sed "s/\b$unwanted\b//")"
        done
    fi

    # replace codenames with actual ones
    # order matters:)
    filtered_build_jobs="${filtered_build_jobs//oldstable/$OLDSTABLE_CODENAME}"
    filtered_build_jobs="${filtered_build_jobs//newstable/$NEWSTABLE_CODENAME}"
    filtered_build_jobs="${filtered_build_jobs//stable/$STABLE_CODENAME}"
    independent_build_jobs="${independent_build_jobs//oldstable/$OLDSTABLE_CODENAME}"
    independent_build_jobs="${independent_build_jobs//newstable/$NEWSTABLE_CODENAME}"
    independent_build_jobs="${independent_build_jobs//stable/$STABLE_CODENAME}"

    # we have system-wide excludes, but those are codename-specific and has to be executed after the 
    # translation we just did above
    # the filter applies only to jobs generated by build_for_product, since those are the only ones
    # who contain product in its name
    if [ "$build_for_product" != "" ] && [ "$product_build_jobs_exclude" != "" ]; then
        for unwanted in $product_build_jobs_exclude; do
            # https://stackoverflow.com/questions/1032023/sed-whole-word-search-and-replace
            filtered_build_jobs="$( echo "$filtered_build_jobs" | sed "s/\b$unwanted\b//")"
        done
    fi
fi # build_exactly_for != ""

# and now decide for which repository the build will be
if [ "$CI_COMMIT_TAG" != "" ]; then
    # build started for tag, building main package
    repository="main"
elif [ "$CI_COMMIT_BRANCH" == "test" ]; then
    # building started on test branch, build test package
    repository="test"
else
    # neither one, probably a dev branch, deploy stage will be suppressed
    repository=""
fi


############################################################################
# recap of the variables

cat << EOF
Recap of the passed and computed variables:

 build_exactly_for = $build_exactly_for
 build_for_product = $build_for_product
 disable_build = $disable_build
 build_for_architecture = $build_for_architecture
 build_for_distribution = $build_for_distribution
 disable_for = $disable_for
 mirror_to_github = $mirror_to_github

 package_name = $package_name
 product_build_jobs_exclude = $product_build_jobs_exclude
 build_elements = $build_elements
 build_distributions = $build_distributions
 repository = $repository
 filtered_build_jobs = $filtered_build_jobs
 independent_build_jobs = $independent_build_jobs
EOF

############################################################################
# generating of the target CI script

# erase possible content of the generated CI file
echo -n "" > "$ci_file"

# generate basic list of stages
echo "stages:
" >> "$ci_file"

# if no repository is set, that means we are building for other branch then test or main
# we will generate special "starter" stage with one manual job
# with this, the build will not start automatically
#if [ "$repository" == "" ]; then
#    echo "- starter
#" >> "$ci_file"
#fi

# and add classic stages
echo "- pre_validation
- pre_build
- build
- post_validation
- notification
- deploy

" >> "$ci_file"

# add tag validation for main branch
if [ "$repository" == "main" ]; then
    echo "validate_tag:
    stage: pre_validation
    image: $CI_REGISTRY/docker/bob-the-builder/$STABLE_CODENAME:latest
    script:
         - /ci-scripts/validate-tag.sh '$CI_COMMIT_TAG'
    variables:
        GIT_STRATEGY: 'none'

" >> "$ci_file"
fi

# generate starter job for dev branches
#if [ "$repository" == "" ]; then
#    echo "starter:
#    stage: starter
#    when: manual
#    image: $CI_REGISTRY/docker/bob-the-builder/$STABLE_CODENAME:latest
#    script:
#        - echo 'I am the firestarter, twisted firestarter!'
#" >> "$ci_file"
#fi

# validate the syntax and semantics of shell scripts, but not for main (should be already checked by development and test branches)
#if [ "$repository" != "main" ]; then
#    echo "shellcheck:
#    stage: pre_validation
#    image: $CI_REGISTRY/docker/bob-the-builder/$STABLE_CODENAME:latest
#    script:
#        - /ci-scripts/validators/shellcheck.sh
#
#" >> "$ci_file"
#fi

# generate build jobs
for job in $filtered_build_jobs; do
    # split the job name to useful parts
    dist="$(echo "$job" | cut -d"-" -f 1)"
    element="$(echo "$job" | cut -d"-" -f 2)"

    # get the architecture for the given element
    if [ "$build_for_product" != "" ] || [ "$build_exactly_for" != "" ]; then
        # $element is product, translate it to architecture
        translation_var="architecture_$element"
        architecture="${!translation_var}"
        if [ "$architecture" == "" ]; then
            echo "The architecture for product $element is not defined, expected variable $translation_var to be present."
            exit 1
        fi
        version_suffix="~$dist-$element"
    elif [ "$build_for_architecture" != "" ]; then
        # $element is the architecture itself
        architecture=$element
        version_suffix="~$dist"
    fi

    echo "$job:
    stage: build
    tags: ['$architecture']
    image: $CI_REGISTRY/docker/bob-the-builder/$dist:latest
    before_script:
        - \"[ -e gitlab-ci/build-before-script.sh ] && . gitlab-ci/build-before-script.sh\"
    script:
        - /ci-scripts/build-package.sh --build=any $package_name
    artifacts:
        paths:
            - build/
    variables:
        VERSION_SUFFIX: '$version_suffix'
        GIT_SUBMODULE_STRATEGY: 'recursive'
        GIT_STRATEGY: 'clone'
        GIT_CHECKOUT: 'true'
        ARCHITECTURE: '$architecture'
" >> "$ci_file"

    # add PRODUCT and PLATFORM variable when building per product
    # PLATFORM is DEPRECATED !
    if [ "$build_for_product" != "" ]; then
        echo "        PRODUCT: '$element'
        PLATFORM: '$element'
" >> "${ci_file}"
    fi

done

# add special job for common packages
for job in $independent_build_jobs; do
    dist="$(echo "$job" | cut -d"+" -f 1)"
    addlink="$(echo "$job" | cut -d"+" -f 2- -s | sed 's/+/ /g')"
    if [ -n "$addlink" ]; then 
        version_suffix=""
    else
        version_suffix="~$dist"
    fi

    echo "independent-$dist:
    stage: build
    tags: ['amd64']
    image: $CI_REGISTRY/docker/bob-the-builder/$dist:latest
    before_script:
        - \"[ -e gitlab-ci/build-before-script.sh ] && . gitlab-ci/build-before-script.sh\"
    script:
        - /ci-scripts/build-package.sh --build=all $package_name
        - echo build $job
    artifacts:
        paths:
            - build/
    variables:
        VERSION_SUFFIX: '$version_suffix'
        GIT_SUBMODULE_STRATEGY: 'recursive'
        GIT_STRATEGY: 'clone'
        GIT_CHECKOUT: 'true'
        BUILD_PKG_LINK: '$addlink'
" >> "$ci_file"
done

# job for sending notification about waiting deploy - only for main branch
if [ "$repository" == "main" ]; then
    echo "slack_notification:
    stage: notification
    image: $CI_REGISTRY/docker/bob-the-builder/$STABLE_CODENAME:latest
    script:
        - \"/ci-scripts/send-notification.sh 'New package \`$package_name\` version \`$CI_COMMIT_TAG\` waits for the deployment to the repository $CI_PROJECT_URL/pipelines/$CI_PIPELINE_ID' PACKAGE_BUILD\"
    variables:
        GIT_STRATEGY: 'none'
" >> "$ci_file"
fi

# generate deploy job based on the target repository
if [ "$repository" == "main" ]; then
echo "deploy-main:
    stage: deploy
    tags: ['amd64']
    image: $CI_REGISTRY/docker/bob-the-builder/$STABLE_CODENAME:latest
    when: manual
    script:
        - /ci-scripts/deploy-packages.sh
    variables:
        GIT_STRATEGY: 'none'
" >> "${ci_file}"
elif [ "$repository" == "test" ]; then
    echo "deploy-test:
    stage: deploy
    tags: ['amd64']
    image: $CI_REGISTRY/docker/bob-the-builder/$STABLE_CODENAME:latest
    script:

        - /ci-scripts/push-tags.sh .test.
        - /ci-scripts/deploy-packages.sh
" >> "$ci_file"
fi

if [ "$mirror_to_github" == "true" ]; then
    echo "mirror_to_github:
    stage: deploy
    tags: ['amd64']
    image: $CI_REGISTRY/docker/bob-the-builder/$STABLE_CODENAME:latest
    when: manual
    before_script:
        - \"[ -e gitlab-ci/mirror-before-script.sh ] && . gitlab-ci/mirror-before-script.sh\"
    script:
        - /ci-scripts/mirror-to-github.sh
    variables:
        GIT_DEPTH: 0
        GIT_STRATEGY: 'clone'
        GIT_SUBMODULE_STRATEGY: 'recursive'
    dependencies: []
" >> "$ci_file"
fi

# add gitlab-ci.yml snippet with additional jobs (e.g. for pre_build stage)
if [ -f "gitlab-ci/gitlab-ci-include.yml" ]; then
    cat "gitlab-ci/gitlab-ci-include.yml" >> "$ci_file"
fi

echo
echo "########################################################################################"
echo
cat "$ci_file"
