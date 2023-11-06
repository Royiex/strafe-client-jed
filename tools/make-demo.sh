mkdir -p ../target/demo
mv ../target/x86_64-pc-windows-gnu/release/strafe-client.exe ../target/demo/strafe-client.exe
rm ../target/demo.7z
7z a -t7z -mx=9 -mfb=273 -ms -md=31 -myx=9 -mtm=- -mmt -mmtf -md=1536m -mmf=bt3 -mmc=10000 -mpb=0 -mlc=0 ../target/demo.7z ../target/demo