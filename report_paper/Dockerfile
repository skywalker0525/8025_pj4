FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    latexmk \
    texlive-latex-base \
    texlive-latex-recommended \
    texlive-latex-extra \
    texlive-pictures \
    texlive-fonts-recommended \
    texlive-bibtex-extra \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /work

CMD ["bash", "build_in_docker.sh"]
